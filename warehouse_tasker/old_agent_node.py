import math

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Time, Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

from tf2_ros import Buffer, TransformListener, transform_listener
from geometry_msgs.msg import TransformStamped

from warehouse_tasker_interfaces.srv import SendGoal, Register, SendTask

from enum import Enum
from dataclasses import dataclass

class State(Enum):
    STANDBY: int    = 1
    ACTIVE: int     = 2
    RETURNING: int  = 3

@dataclass
class ObjectData:
    id:         str
    active:     bool = False

class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name='agent_node')

        # ROS parameter declaration
        self.declare_parameter('goal_count', 16)

        # Object Variables
        self.initial_pose: PoseStamped | None = None

        self.goals: list[ObjectData]    = []
        self.tasks: list[str]           = []

        self.compute_goal: PoseStamped | None = None
        self.active_goal: PoseStamped | None = None
        self.state                      = State.STANDBY

        # Action Clients
        self._compute_action_client     = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_length: float | None  = None

        self._nav_action_client         = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_is_complete: bool      = False

        # Services
        self.task_service               = self.create_service(SendTask, 'send_task', self.send_task_callback)
        # self.goal_service               = self.create_service(SendGoal, 'send_goal', self.nav_goal_callback)
        self.compute_path_service       = self.create_service(SendGoal, 'compute_path_length', self.compute_path_length_callback)

        # Service Clients
        self.registration_client        = self.create_client(Register, '/register_agent')
        self.door_client                = self.create_client(SetBool, 'open_door')

        # Transform buffer and listener to get marker tfs
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        # Register agent to mission node
        # self.namespace: str             = self.get_namespace() if not self.get_namespace == '/' else ''
        # print(f'{self.namespace=}')
        # self.register_agent(self.namespace)

        self.initialise_goals(self.get_parameter('goal_count').get_parameter_value().integer_value)
        self.get_initial_pose()

        self.loop_timer = self.create_timer(0.5, self.loop)
        self.callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------
    
    def initialise_goals(self, number_of_goals: int) -> None:
        """Initialises goals"""
        for goal_index in range(number_of_goals):
            self.goals.append(ObjectData(id=str(goal_index)))

        self.get_logger().info(f'Initialised {number_of_goals} goals.')

    # def register_agent(self, name: str):
    #     while not self.registration_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn(f'{self.registration_client.srv_name} service not available, waiting again...')
    #
    #     req = Register.Request()
    #
    #     req.id = name
    #
    #     self.get_logger().info(f'Registering agent {name}')
    #
    #     future = self.registration_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()

    # HACK: May be able to subscribe to an initial pose topic, but does not right now.
    def get_initial_pose(self):
        # self.attempts = 0
        # while self.initial_pose is None and self.attempts < 11:
        #     if self.attempts == 10:
        #         self.get_logger().error('Could not find tf. Shutting down...')
        #         rclpy.shutdown()

        self.get_logger().info('Finding initial pose...')
        transform = self.get_transform_from_map('odom')
        if transform is None:
            self.get_logger().warn('Could not find tf base_footprint. Trying again...')
            # self.attempts += 1
            # continue
            return

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.pose.position.x = transform.transform.translation.x
        self.initial_pose.pose.position.y = transform.transform.translation.y
        self.initial_pose.pose.position.z = transform.transform.translation.z
        self.initial_pose.pose.orientation.x = transform.transform.rotation.x
        self.initial_pose.pose.orientation.y = transform.transform.rotation.y
        self.initial_pose.pose.orientation.z = transform.transform.rotation.z
        self.initial_pose.pose.orientation.w = transform.transform.rotation.w
        self.get_logger().info('Found initial pose!')

    def activate_payload_mechanism(self):
        while not self.door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.door_client.srv_name} service not available, waiting again...')
        req = SetBool.Request()
        req.data = True

        self.get_logger().info(f'Calling service to payload mechanism...')

        future = self.door_client.call_async(req)
        # FIX: this may be cause for a deadlock...
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    def compute_path_length_callback(self, request: SendGoal.Request, response: SendGoal.Response):
        self.compute_goal = request.goal_pose
        self.compute_goal.header.frame_id = 'map'

        self.goal_id = request.goal_id

        self.get_logger().info(f'Received goal locations - x: {self.compute_goal.pose.position.x}, y: {self.compute_goal.pose.position.y}')

        # TODO: Check what state the robot should be when computing path length
        self.state = State.STANDBY

        response.success = True
        return response

    def nav_goal_callback(self, request: SendGoal.Request, response: SendGoal.Response):
        self.compute_goal = request.goal_pose
        self.compute_goal.header.frame_id = 'map'

        self.goal_id = request.goal_id

        self.get_logger().info(f'Received goal locations - x: {self.compute_goal.pose.position.x}, y: {self.compute_goal.pose.position.y}')

        # TODO: Check what state the robot should be when computing path length
        self.state = State.ACTIVE

        response.success = True
        return response

    def send_task_callback(self, request: SendTask.Request, response: SendTask.Response):
        """Task callback function"""
        requested_goal = next((goal for goal in self.goals if goal.id == str(request.goal)), None)

        if requested_goal is None:
            self.get_logger().warn(f'Incoming task rejected: Non-existent goal {request.goal}!')
            response.success = False
            return response

        if requested_goal.active:
            self.get_logger().warn(f'Incoming task rejected: Goal {request.goal} has already been set!')
            response.success = False
            return response
        
        # set to active for agents and goals
        requested_goal.active = True

        # add ongoing task to track them
        self.tasks.append(requested_goal.id)
        self.get_logger().info(f'Incoming task: Robot to Goal {request.goal}')

        response.success = True
        return response

# -------------------------------------------------------------------------------------------

    def send_compute_goal(self, goal: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal

        self._compute_action_client.wait_for_server()

        self._send_compute_goal_handle = self._compute_action_client.send_goal_async(
            goal_msg, feedback_callback=self.compute_feedback_callback
        )
        self._send_compute_goal_handle.add_done_callback(self.compute_goal_response_callback)

    def compute_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_compute_result_future = goal_handle.get_result_async()
        self._get_compute_result_future.add_done_callback(self.get_compute_result_callback)

    def get_compute_result_callback(self, future):
        result: ComputePathToPose.Result = future.result().result
        if result:
            self.get_logger().info(f'Result: {len(result.path.poses)}')
            path_length = self.calculate_path_distance(result.path)
            self.path_length = path_length
            self.get_logger().info(f'Computed path length: {path_length}')
            
        else:
            self.get_logger().warn(f'Result: is None!')
            self.path_length = None


    def compute_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

# -------------------------------------------------------------------------------------------

    def send_nav_goal(self, goal: PoseStamped) -> None:
        self.get_logger().info('Sending goal to action client...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._nav_action_client.wait_for_server()
        self._nav_goal_handle = self._nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        self._nav_goal_handle.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self.get_nav_result_callback)

    def get_nav_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.nav_is_complete = True

    def nav_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

# -------------------------------------------------------------------------------------------

    def calculate_path_distance(self, path: Path) -> float:
        """Calculates the length of a path."""
        path_distance: float = 0.0
        for i, pose in enumerate(path.poses):
            prev_pose = path.poses[i-1].pose.position
            curr_pose = pose.pose.position
            path_segment_distance = math.sqrt((prev_pose.x - curr_pose.x)**2 + (prev_pose.y - curr_pose.y)**2)
            path_distance += path_segment_distance

        return path_distance

    def get_transform_from_map(self, source_frame: str) -> TransformStamped | None:
        """Gets the transform of a given source frame in reference to the map frame."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=f'{source_frame}',
                time=Time(seconds=0),
                timeout=Duration(seconds=0.5),
            )
            self.get_logger().info(f'Successfully found tf!')
            return transform

        except BaseException as e:
            self.get_logger().warn(f'Error getting transform: {e}')
            return None

    def get_marker_transform(self, index) -> TransformStamped | None:
        """Gets the transform of a given marker index."""
        return self.get_transform_from_map(f'marker{index}')

# -------------------------------------------------------------------------------------------

    def loop(self) -> None:
        # self.get_logger().info(f'Current state is {self.state}')
        if self.initial_pose is None:
            self.get_initial_pose()

        if self.compute_goal is not None:
            self.get_logger().info('Sending compute goal...')
            self.send_compute_goal(self.compute_goal)
            self.compute_goal = None

        if self.active_goal is not None:
            self.get_logger().info('Sending nav goal...')
            self.send_nav_goal(self.active_goal)
            self.active_goal = None

        match self.state:
            case State.STANDBY:
                # HACK: Currently removes tasks immediately
                if len(self.tasks) == 0:
                    self.get_logger().warn('Waiting for goal...')
                    return

                transform = self.get_marker_transform(self.tasks[0])
                if transform is None:
                    self.get_logger().warn('Could not find goal tf. Trying again...')
                    return

                self.active_goal = PoseStamped()
                self.active_goal.header.frame_id = 'map'
                self.active_goal.pose.position.x = transform.transform.translation.x
                self.active_goal.pose.position.y = transform.transform.translation.y
                self.active_goal.pose.position.z = transform.transform.translation.z
                self.active_goal.pose.orientation.x = transform.transform.rotation.x
                self.active_goal.pose.orientation.y = transform.transform.rotation.y
                self.active_goal.pose.orientation.z = transform.transform.rotation.z
                self.active_goal.pose.orientation.w = transform.transform.rotation.w
                self.get_logger().info('Found goal!')

                self.tasks.pop(0)
                self.state = State.ACTIVE
            case State.ACTIVE:
                if not self.nav_is_complete:
                    return

                # NOTE: This needs to be run with a working service active
                # TODO: add a wait or create an action_server for the payload_mechanism...
                # self.activate_payload_mechanism()

                self.active_goal = self.initial_pose
                self.get_logger().info(f'Sending a return goal to home {self.active_goal}...') 

                self.get_logger().info('Changing state from ACTIVE to RETURNING!')
                self.state = State.RETURNING
            case State.RETURNING:

                if not self.nav_is_complete:
                    return
                    
                self.get_logger().info('Changing state from RETURNING to STANDBY!')
                self.state = State.STANDBY

        self.nav_is_complete = False
        
# -------------------------------------------------------------------------------------------

def main(args = None) -> None:
    rclpy.init(args = args)

    node = AgentNode()
    node.get_logger().info('Initialised agent node!')

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
