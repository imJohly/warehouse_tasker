from argparse import RawDescriptionHelpFormatter
from threading import activeCount
import rclpy
from rclpy.clock import Duration, Time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Client, Future, MultiThreadedExecutor

from std_srvs.srv import SetBool
from tf2_ros import Buffer, PoseStamped, TransformListener
from geometry_msgs.msg import TransformStamped

from dataclasses import dataclass

from warehouse_tasker_interfaces.srv import Register, SendTask, SendGoal, SendPathDist

from enum import Enum

# TODO: See if I need a state machine?
class State(Enum):
    AWAIT_TASKS:    int = 1
    PICK_AGENT:     int = 2
    SEND_GOAL:      int = 3

# TODO: Maybe match these with the service interfaces

@dataclass
class ObjectData:
    id:             str
    occupied:       bool = False    # FIX: Does a boolean work for this?

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

@dataclass
class GoalFuture:
    id:             str
    future:         Future | None = None

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # ROS Parameters
        self.declare_parameter('goal_count', 10)

        # Object variables
        self._state: int                        = State.AWAIT_TASKS.value

        self._goals: list[ObjectData]           = []
        self._agents: list[ObjectData]          = []

        self._tasks: list[TaskData]             = []
        self._current_task: TaskData | None     = None      # NOTE: See if I'll need this...
        self._task_count: int                   = 0

        self._calculated_paths: list[PathData]  = []

        # Services
        self._registration_service              = self.create_service(Register, 'register_agent', self.registration_callback)
        self._task_service                      = self.create_service(SendTask, 'send_task', self.task_callback)
        self._path_dist_service                 = self.create_service(SendPathDist, 'send_path_dist', self.path_dist_callback)

        # Service Clients
        self._agent_goal_clients: list[Client]      = []
        self._agent_start_nav_clients: list[Client] = [] 

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
            self._goals.append(ObjectData(id=str(goal_index)))

        self.get_logger().info(f'Initialised {goal_count} goals.')

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

    # FIX: If there is a place for a deadlock. It's here.

    def send_goal(self, agent_id: str, goal_id: str, goal_pose: PoseStamped):
        agent_client = self.get_client_by_agent_id(agent_id, self._agent_goal_clients)

        if agent_client is None:
            self.get_logger().error(f'No client found with agent id {agent_id}')
            return

        self.get_logger().warn(f'Waiting for {agent_client.srv_name}...')
        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_client.srv_name} service not available, waiting again...')

        req = SendGoal.Request()
        req.goal_id = goal_id
        req.goal_pose = goal_pose
        self.get_logger().info(f'Sending Goal {goal_id} request to agent {agent_id}...')

        # FIX: Potentially could store as a object variable...
        # This will probably only work for one robot...
        future = agent_client.call_async(req)

        goal_future = GoalFuture(agent_id, future)
        self._send_goal_futures.append(goal_future)

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def registration_callback(self, request: Register.Request, response: Register.Response):
        if request.id in self._agents:
            self.get_logger().warn(f'Agent {request.id} already registered! Skipping...')
        else:
            new_agent = ObjectData(id=request.id)
            self._agents.append(new_agent)
            self.get_logger().info(f'Successfully registered Agent {request.id}!')

        response.success = True
        return response

    def task_callback(self, request: SendTask.Request, response: SendTask.Response):
        # Receive tasks and store them
        # Request is a list

        response.success = True
        return response

# ----------------------------------------------------------------------------

    # NOTE: HELPER FUNCTIONS

    def get_object_by_id(self, object_id: str, object_list: list[ObjectData]) -> ObjectData | None:
        return next((obj for obj in object_list if obj.id == object_id), None)

    def get_client_by_agent_id(self, agent_id: str, client_list: list[Client]) -> Client | None:
        return next((client for client in client_list if agent_id in client.srv_name), None)

    def get_marker_transform(self, index: int | str) -> TransformStamped | None:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=f'marker{index}',
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

        pass

# ----------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init()

    node = MissionNode()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
