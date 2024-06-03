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
        self._state: int                        = State.AWAIT_TASKS

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

        # Futures
        self._send_goal_futures: list[GoalFuture] = []
        # FIX: change this to the dataclass when i get to it...
        self._start_goal_futures: list[Future] = []

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

    def accept_path(self, agent_id: str):
        agent_client = self.get_client_by_agent_id(agent_id, self._agent_start_nav_clients)

        if agent_client is None:
            self.get_logger().error(f'No client found with agent id {agent_id}')
            return

        self.get_logger().warn(f'Waiting for {agent_client.srv_name}...')
        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_client.srv_name} service not available, waiting again...')

        req = SetBool.Request()
        req.data = True
        self.get_logger().info(f'Requesting to open door...')

        # FIX: future will have to be stored in a list
        future = agent_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()
        
# ----------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def registration_callback(self, request: Register.Request, response: Register.Response):
        if request.id in self._agents:
            self.get_logger().warn(f'Agent {request.id} already registered! Skipping...')
        else:
            new_goal_client = self.create_client(SendGoal, f'{request.id}/send_goal') 
            self._agent_goal_clients.append(new_goal_client)
            self.get_logger().info(f'Registered agent client {new_goal_client.srv_name}')

            new_start_nav_client = self.create_client(SetBool, f'{request.id}/start_nav') 
            self._agent_start_nav_clients.append(new_start_nav_client)
            self.get_logger().info(f'Registered agent client {new_start_nav_client.srv_name}')

            new_agent = ObjectData(id=request.id)
            self._agents.append(new_agent)
            self.get_logger().info(f'Successfully registered Agent {request.id}!')

        response.success = True
        return response

    # HACK: Check if this works. Will it reject agents with no namespace?
    def task_callback(self, request: SendTask.Request, response: SendTask.Response):
        requested_goal = self.get_object_by_id(request.goal, self._goals)
        requested_agent = self.get_object_by_id(request.agent, self._agents)

        if requested_goal is None:
            self.get_logger().error('Incoming task rejected! Requested goal is non-existent!')
            response.success = False
            return response

        if requested_goal.occupied:
            self.get_logger().error('Incoming task rejected! Requested goal is already occupied!')
            response.success = False
            return response

        if requested_agent is None:
            self.get_logger().error('Incoming task rejected! Requested goal is non-existent!')
            response.success = False
            return response

        if requested_agent.occupied:
            self.get_logger().error('Incoming task rejected! Requested agent is already occupied!')
            response.success = False
            return response

        new_task = TaskData(
            id=self._task_count,
            agent_id=request.agent,
            goal_id=request.goal,
        )
        self._tasks.append(new_task)

        response.success = True
        return response

    def path_dist_callback(self, request: SendPathDist.Request, response: SendPathDist.Response):
        new_path_dist = PathData(
            path_length=request.path_length,
            agent_id=request.agent_id,
            goal_id=request.goal_id,
        )

        self._calculated_paths.append(new_path_dist)

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

        match self._state:
            # HACK: Does this have to be a separate state, or do I want it at the beginning...
            case State.AWAIT_TASKS:
                if not self._tasks:
                    self.get_logger().warn(f'Waiting for tasks...')
                    return

                if self._current_task is None:
                    self._current_task = self._tasks[0]
                    return

                self._state = State.PICK_AGENT
                self.get_logger().info('Changing state from AWAIT_TASKS to PICK_AGENT')

            case State.PICK_AGENT:
                # FIX: Will probably have to reset if this ever happens...
                if self._current_task is None:
                    self.get_logger().error('Current Task has been lost...')
                    return

                free_agents: list[ObjectData] = [agent for agent in self._agents if not agent.occupied]

                for agent in free_agents:
                    goal_transform = self.get_marker_transform(self._current_task.goal_id)

                    # FIX: Currently will loop if transform, maybe wanna timeout?
                    if goal_transform is None:
                        self.get_logger().error('Could not find goal transform! Trying again...')
                        return
                    
                    # TODO: Move this into a function.
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.pose.position.x = goal_transform.transform.translation.x
                    goal_pose.pose.position.y = goal_transform.transform.translation.y
                    goal_pose.pose.position.z = goal_transform.transform.translation.z
                    goal_pose.pose.orientation.x = goal_transform.transform.rotation.x
                    goal_pose.pose.orientation.y = goal_transform.transform.rotation.y
                    goal_pose.pose.orientation.z = goal_transform.transform.rotation.z
                    goal_pose.pose.orientation.w = goal_transform.transform.rotation.w
                    
                    # check if future is the correct future for current agent
                    goal_future = next((future for future in self._send_goal_futures if future.id == agent.id), None)
                    if goal_future is None:
                        self.send_goal(agent.id, self._current_task.goal_id, goal_pose)
                        self.get_logger().warn(f'No related future goal found for agent {agent.id}. Trying again...')
                        return
                    if goal_future.future is None:
                        # self.send_goal(agent.id, self._current_task.goal_id, goal_pose)
                        self.get_logger().info(f'Goal {self._current_task.goal_id} was sent to agent {agent.id}!')
                        return
                    if not goal_future.future.result():
                        self.get_logger().warn('Response was bad after sending goal. Trying again...')
                        return

                self._state = State.SEND_GOAL
                self.get_logger().info('Changing State from PICK_AGENT to SEND_GOAL')
            case State.SEND_GOAL:
                self.get_logger().info('SEND_GOAL...')

        # task_agent = self.get_object_by_id(self._current_task.agent_id, self._agents)
        # task_goal = self.get_object_by_id(self._current_task.goal_id, self._goals)
        #
        # if task_agent is None:
        #     self.get_logger().error('Could not find agent requested in task! Deleting Task...')
        #     self._tasks.pop(0)
        #     return
        #
        # if task_goal is None:
        #     self.get_logger().error('Could not find goal requested in task! Deleting Task...')
        #     self._tasks.pop(0)
        #     return
        #
        # # Check for the sigle robot case (no namespace)
        # if self._current_task.agent_id == '' and len(self._agents) == 1:
        #     goal_transform = self.get_marker_transform(task_goal.id)
        #
        #     # HACK: Currently will loop if transform, maybe wanna timeout?

        #     if goal_transform is None:
        #         self.get_logger().error('Could not find goal transform! Trying again...')
        #         return
        #     
        #     # TODO: Move this into a function.

        #     goal_pose = PoseStamped()
        #     goal_pose.header.frame_id = 'map'
        #     goal_pose.pose.position.x = goal_transform.transform.translation.x
        #     goal_pose.pose.position.y = goal_transform.transform.translation.y
        #     goal_pose.pose.position.z = goal_transform.transform.translation.z
        #     goal_pose.pose.orientation.x = goal_transform.transform.rotation.x
        #     goal_pose.pose.orientation.y = goal_transform.transform.rotation.y
        #     goal_pose.pose.orientation.z = goal_transform.transform.rotation.z
        #     goal_pose.pose.orientation.w = goal_transform.transform.rotation.w
        #
        #     
        #     if self._goal_future is None:
        #         self.send_goal(task_agent.id, task_goal.id, goal_pose)
        #         return
        # 
        #     self.get_logger().info(f'{self._goal_future.result()=}')
        #
        #     # self.get_logger().info(f'Sent goal {task_goal.id} to agent {task_agent.id}')

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
