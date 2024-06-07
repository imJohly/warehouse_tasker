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
        self._current_pose: PoseStamped | None  = None

        self._current_goal: PoseStamped | None  = None
        self._current_goal_id: str | None       = None
        self._agent_state: int                  = State.STANDBY.value

        self._path_accepted: bool | None        = None

        # Subscribers
        # FIX: Needs to be remapped to tb1? unless cause it relative
        self._initial_pose_subscriber           = self.create_subscription(PoseStamped, 'initial_pose', self.initial_pose_callback, qos_profile=5)
        self._pose_subscriber                   = self.create_subscription(PoseStamped, 'amcl_pose', self.pose_callback, qos_profile=100)

        # Services
        self._goal_service                      = self.create_service(SendGoal, 'send_goal', self.send_goal_callback)

        # Service Clients
        self._registration_client               = self.create_client(Register, '/register_agent')
        self._door_client                       = self.create_client(SetBool, '/open_door')

        # Register agent to mission
        self._namespace                         = self.get_namespace() if not self.get_namespace() == '/' else ''

        if self.get_parameter('use_mission').value:
            self.register_agent(self._namespace)

        # Start main loop
        self.create_timer(1.0, self.main_loop)
        self._callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------

    def initial_pose_callback(self, msg: PoseStamped) -> None:
        """Callback function for receiving an initial pose."""
        self._initial_pose = msg
        self.get_logger().info('Receieved new intial pose!')

    def pose_callback(self, msg: PoseStamped) -> None:
        """Callback function for receiving the current pose."""
        self._current_pose = msg
        self.get_logger().info('Receieved new current pose!')

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

        future = self._door_client.call_async(req)
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

# -------------------------------------------------------------------------------------------

    # NOTE: HELPER FUNCTIONS

    def calculate_distance_between_poses(self, pose_one: PoseStamped, pose_two: PoseStamped) -> float:
        """Calculates the distance between two poses."""
        return math.sqrt((pose_two.pose.position.x - pose_one.pose.position.x) ** 2 +
                         (pose_two.pose.position.y - pose_one.pose.position.y) ** 2)

# -------------------------------------------------------------------------------------------

    def main_loop(self) -> None:
        if self._current_goal is None:
            self.get_logger().warn(f'No goal set, awaiting new goal...')
            return

        

def main(args=None) -> None:
    rclpy.init(args=args)

    node = AgentNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

