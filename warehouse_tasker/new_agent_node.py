import math

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from nav2_msgs.action import NavigateToPose

from std_srvs.srv import SetBool
from warehouse_tasker_interfaces.srv import Register, SendPose

class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name='agent_node')

        # ROS Parameters
        self.declare_parameter('use_mission', True)
        self.declare_parameter('use_door', False)

        # Object Variables
        self.current_state: AgentState          = IdleState(self)

        self._initial_pose: PoseWithCovarianceStamped | None  = None
        self._current_pose: PoseWithCovarianceStamped | None  = None
        self._stored_goals: list[Pose]          = []
        self._current_goal: Pose | None         = None

        # Topic Publishers
        self._state_publisher                   = self.create_publisher(Bool, 'agent_state', qos_profile=10)

        # Topic Subscribers
        self._initial_pose_subscriber           = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, qos_profile=5)

        # Services
        self._goal_service                      = self.create_service(SendPose, 'send_goal', self.send_goal_callback)

        # Service Clients
        self._registration_client               = self.create_client(Register, '/register_agent')
        self._door_client                       = self.create_client(SetBool, 'open_door')

        # Action Clients
        self._nav_action_client                 = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_is_complete: bool             = False

        # Register agent to mission
        self._namespace                         = self.get_namespace() if not self.get_namespace() == '/' else ''

        if self.get_parameter('use_mission').value:
            self.register_agent(self._namespace)

        # Start main loop
        self.create_timer(1.0, self.run)
        self._callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------

    def transition_to(self, new_state) -> None:
        """Transitions the current state of the Mission node.
    
        Arguments:
            new_state - The new state to transition into

        Returns:
            None
        """
        self.current_state.on_exit()
        self.current_state = new_state
        self.current_state.on_enter()
 
    def run(self) -> None:
        """Begins state machine execution"""
        self.current_state.execute()

    def calculate_distance_between_poses(self, pose_one: PoseStamped, pose_two: PoseStamped) -> float:
        """Calculates the distance between two poses.

        Arguments:
            pose_one - first pose
            pose_two - second pose

        Returns:
            The distance as a float between pose_one and pose_two
        """
        return math.sqrt((pose_two.pose.position.x - pose_one.pose.position.x) ** 2 +
                         (pose_two.pose.position.y - pose_one.pose.position.y) ** 2)

# -------------------------------------------------------------------------------------------

    # NOTE: TOPIC CALLBACK FUNCTIONS

    # FIX: Currently doesn't receive anything unless started before turtlebots are launched
    # and until a 2D pose estimate is given.
    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Callback function for receiving an initial pose."""
        self._initial_pose = msg
        self.get_logger().info('Receieved new intial pose!')

# -------------------------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

    def register_agent(self, namespace):
        """Registers agent to mission control with a namespace"""
        attempts = 0
        MAX_ATTEMPTS = 10

        self.get_logger().warn(f'Waiting for {self._registration_client.srv_name}...')
        while not self._registration_client.wait_for_service(timeout_sec=1.0):
            if attempts >= MAX_ATTEMPTS:
                self.get_logger().error('Failed to register agent to mission node! Running without mission...')
                return

            self.get_logger().warn(f'{self._registration_client.srv_name} service not available, attempting another {MAX_ATTEMPTS - attempts} time(s)...')
            attempts += 1

        req = Register.Request()
        req.id = namespace
        self.get_logger().info(f'Registering agent {namespace}')

        future = self._registration_client.call_async(req)
        rclpy.spin_until_future_complete(self, future) 
        return future.result()

    def activate_payload_mechanism(self):
        """Service call to activate payload_system"""
        self.get_logger().warn(f'Waiting for {self._door_client.srv_name}...')
        while not self._door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self._door_client.srv_name} service not available, waiting again...')

        req = SetBool.Request()
        req.data = True
        self.get_logger().info(f'Requesting to open door')

        self._door_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

# -------------------------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def send_goal_callback(self, request: SendPose.Request, response: SendPose.Response):
        if request.pose is None:
            self.get_logger().error('No goal pose was set in request! Ignoring goal...')
            response.success = False
            return response

        # HACK: Check if this actually concatenates the lists
        self._stored_goals += request.pose

        self.get_logger().info(f'Received {len(request.pose)} goal')

        response.success = True
        return response

# -------------------------------------------------------------------------------------------

    # NOTE: ACTION CLIENT

    def start_nav_goal_action(self, goal: Pose) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = goal

        self.get_logger().info(f'Waiting for {self._nav_action_client._action_name} action server to be ready...')
        while not self._nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{self._nav_action_client._action_name} action server is not available. Waiting...')

        self._nav_goal_handle = self._nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        self._nav_goal_handle.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().error('Goal accepted :)')
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self.get_nav_result_callback)

    def get_nav_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().error(f'Navigation Complete!')
        self._nav_is_complete = True

    def nav_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

# -------------------------------------------------------------------------------------------
        
class AgentState:
    def __init__(self, node) -> None:
        self.node: AgentNode = node

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def execute(self):
        pass

class IdleState(AgentState):
    def execute(self):
        self.node._state_publisher.publish(Bool(data=False))

        if self.node._stored_goals is None or len(self.node._stored_goals) <= 0:
            self.node.get_logger().warn(f'Idle: No goals received, awaiting new goal...')
            return

        if self.node._current_goal is not None:
            self.node.get_logger().warn(f'Idle: Goal already set, transitioning to ActiveState...')
            self.node.transition_to(ActiveState(self.node))
            return

        self.node._current_goal = self.node._stored_goals[0]
        self.node._stored_goals.pop(0)
        self.node.get_logger().info(f'Idle: Setting goal {self.node._current_goal.position} as current, transitioning to ActiveState...')

        self.node.transition_to(ActiveState(self.node))

class ActiveState(AgentState):
    def on_enter(self):
        if self.node._current_goal is None:
            self.node.get_logger().warn(f'Active: No goal set, transitioning to IdleState...')
            self.node.transition_to(IdleState(self.node))
            return

        self.node._nav_is_complete = False
        self.node.start_nav_goal_action(self.node._current_goal)
        self.node._current_goal = None

    def execute(self):
        self.node._state_publisher.publish(Bool(data=True))

        if not self.node._nav_is_complete:
            self.node.get_logger().info('Execute: Waiting for navigation to complete...')
            return

        # NOTE: only uses door when parameter set, default doesn't activate.
        if self.node.get_parameter('use_door').value:
            self.node.activate_payload_mechanism()

        if len(self.node._stored_goals) > 0:
            self.node.get_logger().info('Execute: More goals to complete...')
            self.node.transition_to(IdleState(self.node))
            return

        self.node.transition_to(ReturnState(self.node))

class ReturnState(AgentState):
    def on_enter(self):
        if self.node._initial_pose is None:
            self.node.get_logger().info('Return: No initial pose set, not sending return command...')
            return

        initial_pose = Pose()
        initial_pose.position = self.node._initial_pose.pose.pose.position
        initial_pose.orientation = self.node._initial_pose.pose.pose.orientation

        self.node._current_goal = initial_pose

        self.node._nav_is_complete = False
        self.node.start_nav_goal_action(self.node._current_goal)
        self.node._current_goal = None

    def execute(self):
        self.node._state_publisher.publish(Bool(data=False))

        if not self.node._nav_is_complete:
            self.node.get_logger().info('Return: Waiting for return to complete...')

        self.node.get_logger().info('Return: Complete, going to idle state.')
        self.node.transition_to(IdleState(self.node))

# -------------------------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = AgentNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

