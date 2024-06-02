import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

from warehouse_tasker_interfaces.srv import SendGoal, Register, SendPathLength, GetState

from enum import Enum

class State(Enum):
    STANDBY: int    = 1
    ACTIVE: int     = 2
    RETURNING: int  = 3

class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name='agent_node')

        self.compute_goal: PoseStamped | None = None
        self.active_goal: PoseStamped | None = None
        self.state                      = State.STANDBY

        # Action Clients
        self._compute_action_client     = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_length: float | None  = None

        self._nav_action_client         = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_is_complete: bool      = False

        # Services
        self.goal_service               = self.create_service(SendGoal, 'send_goal', self.nav_goal_callback)
        self.compute_path_service       = self.create_service(SendGoal, 'compute_path_length', self.compute_path_length_callback)

        # Service Clients
        self.registration_client        = self.create_client(Register, '/register_agent')
        self.state_client               = self.create_client(GetState, '/get_agent_state')
        self.send_path_length_client    = self.create_client(SendPathLength, '/send_path_length')
        self.door_client                = self.create_client(SetBool, 'open_door')

        # Register agent to mission node
        self.namespace: str             = self.get_namespace() if not self.get_namespace == '/' else ''
        print(f'{self.namespace=}')
        self.register_agent(self.namespace)

        self.loop_timer = self.create_timer(0.5, self.loop)
        self.callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------

    def register_agent(self, name: str):
        while not self.registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.registration_client.srv_name} service not available, waiting again...')

        req = Register.Request()

        req.id = name

        self.get_logger().info(f'Registering agent {name}')

        future = self.registration_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_state(self, state: State):
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.state_client.srv_name} service not available, waiting again...')

        req = GetState.Request()

        req.agent = self.namespace
        req.state = state.value

        self.get_logger().info(f'Sending agent state: {self.state}')

        # FIX: future result currently causes a deadlock...
        future = self.state_client.call_async(req)
        return future.result()        

    def send_path_length(self, path_length: float):
        while not self.send_path_length_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.send_path_length_client.srv_name} service not available, waiting again...')

        req = SendPathLength.Request()

        req.path_length = path_length
        req.agent = self.namespace
        req.goal = self.goal_id

        self.get_logger().info(f'Sending path length of: {path_length}')

        # FIX: future result currently causes a deadlock...
        future = self.send_path_length_client.call_async(req)
        return future.result()

    def activate_payload_mechanism(self):
        while not self.door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.door_client.srv_name} service not available, waiting again...')
        req = SetBool.Request()
        req.data = True

        self.get_logger().info(f'Calling service to payload mechanism...')

        #:FIX: this may be cause for a deadlock...
        future = self.door_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    def compute_path_length_callback(self, request: SendGoal.Request, response: SendGoal.Response):
        self.compute_goal = PoseStamped()
        self.compute_goal.header.frame_id = 'map'
        self.compute_goal.pose.orientation.w = 1.0

        self.compute_goal.pose.position.x = request.x
        self.compute_goal.pose.position.y = request.y

        self.goal_id = request.goal_id

        self.get_logger().info(f'Received goal locations - x: {request.x}, y: {request.y}')

        # TODO: Check what state the robot should be when computing path length
        self.state = State.STANDBY

        response.success = True
        return response

    def nav_goal_callback(self, request: SendGoal.Request, response: SendGoal.Response):
        self.active_goal = PoseStamped()
        self.active_goal.header.frame_id = 'map'
        self.active_goal.pose.orientation.w = 1.0

        self.active_goal.pose.position.x = request.x
        self.active_goal.pose.position.y = request.y

        self.get_logger().info(f'Receive goal locations - x: {request.x}, y: {request.y}')
        self.state = State.ACTIVE

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

# -------------------------------------------------------------------------------------------

    def loop(self) -> None:
        self.get_logger().info(f'Current state is {self.state}')

        if self.compute_goal is not None:
            self.get_logger().info('Sending compute goal...')
            self.send_compute_goal(self.compute_goal)
            self.compute_goal = None

        if self.active_goal is not None:
            self.get_logger().info('Sending nav goal...')
            self.send_nav_goal(self.active_goal)
            self.active_goal = None

        if self.path_length is not None:
            self.get_logger().info('Sent path length request to mission node')
            self.send_path_length(self.path_length)
            self.path_length = None

        if not self.nav_is_complete:
            return

        # change agent state once action is complete
        match self.state:
            case State.STANDBY:
                # FIX: This is wrong, needs to send the path distance
                pass
            case State.ACTIVE:
                # NOTE: This needs to be run with a working service active
                # self.activate_payload_mechanism()

                # TODO: add a wait or create an action_server for the payload_mechanism...

                # TODO: need to get initial pose and set that as the return position
                self.active_goal = PoseStamped()
                self.active_goal.header.frame_id = 'map'
                self.active_goal.pose.position.x = -1.0
                self.active_goal.pose.position.y = -3.0
                self.active_goal.pose.orientation.w = 1.0
                self.nav_is_complete = False

                self.get_logger().info('Sending a return goal to home...') 

                self.state = State.RETURNING
            case State.RETURNING:
                self.state = State.STANDBY

        self.nav_is_complete = False
        
        # TODO: Send service call to mission node to tell it has change state??? or topic.
        self.send_state(self.state)

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
