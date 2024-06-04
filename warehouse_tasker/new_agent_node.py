import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import ActionClient, PoseStamped

from nav2_msgs.action import ComputePathToPose, NavigateToPose

from nav_msgs.msg import Path

from std_srvs.srv import SetBool
from warehouse_tasker_interfaces.srv import AgentState, Register, SendGoal, SendPathDist

from enum import Enum

class State(Enum):
    STANDBY:    int = 1
    COMP_PATH:  int = 2
    ACTIVE:     int = 3
    ARRIVED:    int = 4
    RETURNING:  int = 5

class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__('agent_node')

        # ROS Parameters
        self.declare_parameter('use_mission', True)

        # Object Variables
        self._initial_pose: PoseStamped | None  = None
        self._current_goal: PoseStamped | None  = None
        self._current_goal_id: str | None       = None
        self._agent_state: int                  = State.STANDBY

        self._path_accepted: bool | None        = None

        # Action Clients
        self._compute_action_client             = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._computed_path: Path | None        = None

        self._nav_action_client                 = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_is_complete: bool             = False

        # Services
        self._goal_service                      = self.create_service(SendGoal, 'send_goal', self.send_goal_callback)
        self._nav_service                       = self.create_service(SetBool, 'start_nav', self.start_nav_callback)

        # Service Clients
        self._registration_client   = self.create_client(Register, '/register_agent')
        self._door_client           = self.create_client(SetBool, '/open_door')
        self._path_distance_client  = self.create_client(SendPathDist, '/send_path_dist')

        # Register agent to mission on initialisation
        self._namespace             = self.get_namespace() if not self.get_namespace() == '/' else ''
        if self.get_parameter('use_mission').value:
            self.register_agent(self._namespace)

        # Start main loop
        self.create_timer(1.0, self.main_loop)
        self._callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

        # NOTE: spin_until_future_complete() works here only
        # because it is not called within a callback function!

    def register_agent(self, namespace):
        self.get_logger().warn(f'Waiting for {self._registration_client.srv_name}...')
        while not self._registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self._registration_client.srv_name} service not available, waiting again...')

        req = Register.Request()
        req.id = namespace
        self.get_logger().info(f'Registering agent {namespace}')

        future = self._registration_client.call_async(req)
        rclpy.spin_until_future_complete(self, future) 
        return future.result()

    # FIX: PLEASE CHECK IF THESE ARE ALLOWED TO SPIN UNTIL FUTURE COMPLETE, AS THESE ARE RUN IN MAIN LOOP!

    def open_door(self):
        self.get_logger().warn(f'Waiting for {self._door_client.srv_name}...')
        while not self._door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self._door_client.srv_name} service not available, waiting again...')

        req = SetBool.Request()
        req.data = True
        self.get_logger().info(f'Requesting to open door')

        future = self._door_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_path_dist(self, distance: float, goal_id: str):
        self.get_logger().warn(f'Waiting for {self._path_distance_client.srv_name}...')
        while not self._path_distance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self._path_distance_client.srv_name} service not available, waiting again...')

        # TODO: Change request to correct name of 'path_dist'
        req = SendPathDist.Request()
        req.path_length = distance
        req.goal_id     = goal_id

        # FIX: CHECK IF THIS SPINS CORRECTLY!!!
        future = self._path_distance_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def send_goal_callback(self, request: SendGoal.Request, response: SendGoal.Response):
        if request.goal_pose is None:
            self.get_logger().error('No goal pose was set in request! Ignoring goal...')
            response.success = False
            return response

        if request.goal_id is None:
            self.get_logger().error('No goal ID given in request! Ignoring goal...')
            response.success = False
            return response

        self._current_goal      = request.goal_pose
        self._current_goal_id   = request.goal_id

        self.get_logger().info(f'Received goal locations - x: {request.goal_pose.pose.position.x}, y: {request.goal_pose.pose.position.y}')

        response.success = True
        return response

    def start_nav_callback(self, request: SetBool.Request, response: SetBool.Response):
        self._path_accepted = request.data
        response.success = True
        return response

# -------------------------------------------------------------------------------------------

    # NOTE: ACTION CLIENT

    def start_compute_goal_action(self, goal: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal

        self.get_logger().info(f'Waiting for {self._compute_action_client._action_name} action server to be ready...')
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{self._compute_action_client._action_name} action server is not available. Waiting...')

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
            self.get_logger().info(f'Resulting path poses: {len(result.path.poses)}')
            self._computed_path = result.path
        else:
            self.get_logger().warn(f'Result: is None!')
            self._computed_path = None

    def compute_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

# -------------------------------------------------------------------------------------------

    # NOTE: ACTION CLIENT

    def start_nav_goal_action(self, goal: PoseStamped) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        # NOTE: Reset goal here!
        self._current_goal = None

        self.get_logger().info(f'Waiting for {self._nav_action_client._action_name} action server to be ready...')
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{self._nav_action_client._action_name} action server is not available. Waiting...')

        self._nav_goal_handle = self._nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )

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

    # NOTE: HELPER FUNCTIONS

    def calculate_distance_between_poses(self, pose_one: PoseStamped, pose_two: PoseStamped) -> float:
        """Calculates the distance between two poses."""
        return math.sqrt((pose_two.pose.position.x - pose_one.pose.position.x) ** 2 +
                         (pose_two.pose.position.y - pose_one.pose.position.y) ** 2)

    def calculate_path_distance(self, path: Path) -> float:
        """Calculates the distance of a path."""
        path_distance: float = 0.0
        for i, pose in enumerate(path.poses):
            prev_pose = path.poses[i-1]
            curr_pose = pose
            path_segment_distance = self.calculate_distance_between_poses(curr_pose, prev_pose)
            path_distance += path_segment_distance

        return path_distance

    def reset_state(self) -> None:
        self._agent_state       = State.STANDBY
        self._current_goal      = None
        self._current_goal_id   = None
        self._path_accepted     = False
        self._nav_is_complete   = False

# -------------------------------------------------------------------------------------------

    def main_loop(self) -> None:
        # self.get_logger().info(f'Current agent state is set to {self._agent_state}')

        if self._current_goal is None:
            self.get_logger().warn(f'No goal set, awaiting new goal...')
            return
        
        if self._current_goal_id is None:
            self.get_logger().warn(f'No goal ID set, check goal is set correctly. Standing by...')
            return

        # TODO: Test states correctly transition...
        match self._agent_state:
            case State.STANDBY:
                self.get_logger().info(f'Computing path to goal...')
                self.start_compute_goal_action(self._current_goal)
                
                self._agent_state = State.COMP_PATH
                self.get_logger().info('Computing path. Changing state from STANDBY to COMP_PATH!')

            case State.COMP_PATH:
                if self._computed_path is None:
                    self.get_logger().warn('Computed path is None! Trying Again...')
                    return

                self.get_logger().info(f'Sending path distance to mission...')
                path_dist: float = self.calculate_path_distance(self._computed_path)
                ok = self.send_path_dist(path_dist, self._current_goal_id)

                # HACK: DELETE ONCE TESTED
                print(f'YAY resulting data from sending path distance: {ok}')

                if not ok:
                    self.get_logger().error('Failed to send path distance to mission node! Trying again...')
                    return

                if self._path_accepted is None:
                    self.get_logger().info('Waiting for response from Mission node...')
                    return

                if not self._path_accepted:
                    self.get_logger().info('Mission node rejected path. Resetting state to STANDBY...')
                    self.reset_state()
                    return
                
                self._agent_state = State.ACTIVE
                self.get_logger().info('Mission node accepted path. Changing state from COMP_PATH to ACTIVE!')
                
            case State.ACTIVE:
                if self._current_goal is None:
                    self._agent_state = State.STANDBY
                    self.get_logger().warn(f'No goal set, going back to STANDBY state...')
                    return

                self.start_nav_goal_action(self._current_goal)

                # HACK: May be a point of bugs... need to test.
                if  self._nav_is_complete:
                    self._agent_state = State.ARRIVED
                    self.get_logger().info('Changing state from ACTIVE to ARRIVED!')
            
            case State.ARRIVED:
                self.get_logger().info(f'Opening door...')
                ok = self.open_door()

                if not ok:
                    self.get_logger().error('Failed to open door! Trying again...')
                    return

                self._agent_state = State.ACTIVE
                self.get_logger().info('Changing state from ARRIVED to RETURNING!')

            case State.RETURNING:
                if self._initial_pose is None:
                    self.get_logger().error(f'No initial pose set, unable to return home...')
                    return

                if self._agent_state is None:
                    self._agent_state = State.STANDBY
                    self.get_logger().warn(f'No goal set, going back to STANDBY state...')
                    return

                self.start_nav_goal_action(self._initial_pose)

                # HACK: May be a point of bugs... need to test.
                if self._nav_action_client:
                    self._agent_state = State.STANDBY
                    self.get_logger().info('Changing state from RETURNING to STANDBY!')

def main(args=None) -> None:
    rclpy.init(args=args)

    node = AgentNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

