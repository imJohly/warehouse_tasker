import rclpy
from rclpy.clock import Duration, Time
from rclpy.node import Node, Subscription
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Client, Future, MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped

from dataclasses import dataclass

from warehouse_tasker_interfaces.srv import Register, SendTask, SendPose, GetPose
from enum import Enum

# TODO: See if I need a state machine?
class State(Enum):
    STANDBY = 1
    DISTANCE = 2
    SEQUENCE = 3
    SEND_GOAL = 4

@dataclass
class ObjectData:
    id:             str
    pose:           Pose
    occupied:       bool = False

@dataclass
class TaskData:
    id:             int
    agent_id:       str
    goal_id:        str
    calculated:     bool = False    # FIX: This should probably be something else, STATE or COMPLETED?

@dataclass
class PathData:
    path_length:    float
    agent_id:       str
    goal_id:        str

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # ROS Parameters
        self.declare_parameter('goal_count', 10)

        # Object variables
        self._state                             = State.value

        self._goals: dict[str, ObjectData]      = {}
        self._agents: dict[str, ObjectData]     = {}

        self._tasks: list[TaskData]             = []
        self._task_count: int                   = 0

        # Subscribers
        self._agent_pose_subscribers: list[Subscription] = []

        # Services
        self._registration_service              = self.create_service(Register, 'register_agent', self.registration_callback)
        self._task_service                      = self.create_service(SendTask, 'send_task', self.task_callback)

        # Service Clients
        self._agent_goal_clients: dict[str, Client] = {}
        self._agent_pose_clients: dict[str, Client] = {}
        self._agent_pose_results: dict[str, Future] = {}

        # Transform buffer and listener to get marker tfs
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.initialise_goals(self.get_parameter('goal_count').get_parameter_value().integer_value)
        self.loop_timer = self.create_timer(1.0, self.main_loop)
        self.callback_group = ReentrantCallbackGroup()

# ----------------------------------------------------------------------------

    def initialise_goals(self, goal_count: int) -> None:
        if goal_count <= 0:
            self.get_logger().error('Failed to initialise goals! Shutting down...')
            rclpy.shutdown()
            return

        for goal_index in range(goal_count):
            goal_tf = self.get_marker_transform(f'{goal_index}')

            if goal_tf is None:
                self.get_logger().warn(f'Failed to initialise goal {goal_index}, could not find goal tf!')
                continue

            goal_pose = Pose()
            goal_pose.position = goal_tf.transform.translation
            goal_pose.orientation = goal_tf.transform.rotation

            self._goals[f'goal_index'] = ObjectData(id=str(goal_index), pose=goal_pose)

        self.get_logger().info(f'Initialised {goal_count} goals.')

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def registration_callback(self, request: Register.Request, response: Register.Response):
        if request.id in self._agents:
            self.get_logger().warn(f'Agent {request.id} already registered! Skipping...')
        else:
            new_goal_client = self.create_client(SendPose, f'{request.id}/send_goal')
            self._agent_goal_clients[f'{request.id}'] = new_goal_client
            new_pose_client = self.create_client(GetPose, f'{request.id}/get_pose')
            self._agent_pose_clients[f'{request.id}'] = new_pose_client

            new_agent = ObjectData(id=request.id)
            self._agents.append(new_agent)
            self.get_logger().info(f'Successfully registered Agent {request.id} and its services!')

        response.success = True
        return response

    def task_callback(self, request: SendTask.Request, response: SendTask.Response):
        # Receive tasks and store them
        # Request is a list

        response.success = True
        return response

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

    def send_goal(self, agent_id: str, goal_id: str, goal_pose: PoseStamped) -> None:
        agent_client = self._agent_goal_clients[f'{agent_id}']
        if agent_client is None:
            self.get_logger().error(f'No client found with agent id {agent_id}')
            return

        self.get_logger().warn(f'Waiting for {agent_client.srv_name}...')
        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_client.srv_name} service not available, waiting again...')

        req = SendPose.Request()
        req.id = goal_id
        req.pose = goal_pose
        self.get_logger().info(f'Sending Goal {goal_id} request to agent {agent_id}...')

        agent_client.call_async(req)
      
    def get_agent_pose(self, agent_id: str) -> None:
        if agent_id not in self._agent_pose_clients.keys():
            self.get_logger().error(f'No client found with agent id {agent_id}')
            return
        agent_client = self._agent_pose_clients[agent_id]

        self.get_logger().warn(f'Waiting for {agent_client.srv_name}...')
        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_client.srv_name} service not available, waiting again...')

        req = GetPose.Request()
        req.id = agent_id

        self.get_logger().info(f'Sending pose request to agent {agent_id}...')
        future = agent_client.call_async(req)
        future.add_done_callback(self.future_pose_callback)

    def future_pose_callback(self, future: Future):
        try:
            response: PoseWithCovarianceStamped | None = future.result()
            if response is None:
                return
            self.get_logger().info(f'Pose: {response.pose.pose.pose.position}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


# ----------------------------------------------------------------------------

    # NOTE: HELPER FUNCTIONS

    def get_marker_transform(self, id: str) -> TransformStamped | None:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=f'marker{id}',
                time=Time(seconds=0),
                timeout=Duration(seconds=1),
            )
            self.get_logger().info(f'Successfully found tf!')
            return transform

        except BaseException as e:
            self.get_logger().warn(f'Error getting transform: {e}')
            return None

# ----------------------------------------------------------------------------

    def main_loop(self) -> None:

        if len(self._tasks) <= 0:
            self.get_logger().info('Waiting for tasks...')
            return

# ----------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = MissionNode()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
