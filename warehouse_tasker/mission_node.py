import argparse
from dataclasses import dataclass

import rclpy
from rclpy.clock import Time
from rclpy.duration import Duration
from rclpy.node import Client, Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from warehouse_tasker_interfaces.srv import SendPathLength, SendTask, SendTaskAny, Register, SendGoal, GetState

from enum import Enum

class State(Enum):
    STANDBY: int    = 1
    ACTIVE: int     = 2
    RETURNING: int  = 3

@dataclass
class ObjectData:
    id:         str
    active:     bool = False

@dataclass
class TaskData:
    id:         int
    agent_id:   str
    goal_id:    str
    calculated: bool = False

@dataclass
class PathData:
    path_length:    float
    agent_id:       str
    goal_id:        str

class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')
        
        # ROS parameter declaration
        # NOTE: This can only be changed in launch, currently defaults correctly for sim world.
        self.declare_parameter('goal_count', 16)

        # Object variables
        self.goals: list[ObjectData]    = []
        self.agents: list[ObjectData]   = []

        self.tasks: list[TaskData]      = []
        self.task_count: int            = 0

        self.calculated_paths: list[PathData] = []

        # Services
        self.registration_service       = self.create_service(Register, 'register_agent', self.registration_callback)
        self.state_service              = self.create_service(GetState, 'get_agent_state', self.agent_state_callback)
        self.task_service               = self.create_service(SendTask, 'send_task', self.send_task_callback)
        self.task_any_service           = self.create_service(SendTaskAny, 'send_task_any', self.send_task_any_callback)
        self.send_path_length_service   = self.create_service(SendPathLength, 'send_path_length', self.send_path_length_callback)

        # Clients
        self.agent_compute_goal_clients: list [Client]  = []
        self.agent_nav_goal_clients: list[Client]       = []

        # Transform buffer and listener to get marker tfs
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.initialise_goals(self.get_parameter('goal_count').get_parameter_value().integer_value)
        self.loop_timer = self.create_timer(0.5, self.loop)
        self.callback_group = ReentrantCallbackGroup()

# ----------------------------------------------------------------------------

    def initialise_goals(self, number_of_goals: int) -> None:
        """Initialises goals"""
        for goal_index in range(number_of_goals):
            self.goals.append(ObjectData(id=str(goal_index)))

        self.get_logger().info(f'Initialised {number_of_goals} goals.')

# ----------------------------------------------------------------------------

    def registration_callback(self, request: Register.Request, response: Register.Response):
        """Agent registration callback function"""
        self.agents.append(ObjectData(id=request.id))

        # create new clients for registered agent
        # FIX: Needs to check if it is a valid namespace
        new_compute_client = self.create_client(SendGoal, f'{request.id}/compute_path_length') 
        self.agent_compute_goal_clients.append(new_compute_client)
        self.get_logger().info(f'Registered agent client {new_compute_client.srv_name}')
        
        new_nav_client = self.create_client(SendGoal, f'{request.id}/send_goal') 
        self.agent_nav_goal_clients.append(new_nav_client)
        self.get_logger().info(f'Registered agent client {new_nav_client.srv_name}')

        self.get_logger().info(f'Registered agent {request.id}')

        response.success = True
        return response

    def agent_state_callback(self, request: GetState.Request, response: GetState.Response):
        requested_agent = next((agent for agent in self.agents if agent.id == request.agent), None)

        if requested_agent is None:
            self.get_logger().warn(f'Incoming state change rejected: Non-existent agent {request.agent}!')
            response.success = False
            return response

        # HACK: should probably make this better... 'active' and 'returning' states redundant
        if request.state == State.STANDBY.value:
            requested_agent.active = False

        self.get_logger().info(f'Agent {requested_agent.id} of state {request.state} has active set to {requested_agent.active}')

        response.success = True
        return response

    def send_task_any_callback(self, request: SendTaskAny.Request, response: SendTaskAny.Response):
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
        # requested_agent.active = True
        requested_goal.active = True

        # add ongoing task to track them
        self.task_count += 1
        new_task = TaskData(
            id=self.task_count,
            agent_id='',
            goal_id=requested_goal.id,
        )
        self.tasks.append(new_task)
        self.get_logger().info(f'Incoming task: Goal {request.goal} needs pickup!')

        response.success = True
        return response

    def send_task_callback(self, request: SendTask.Request, response: SendTask.Response):
        """Task callback function"""
        requested_agent = next((agent for agent in self.agents if agent.id == str(request.agent)), None)
        requested_goal = next((goal for goal in self.goals if goal.id == str(request.goal)), None)

        if requested_agent is None:
            self.get_logger().warn(f'Incoming task rejected: Non-existent agent {request.agent}!')
            response.success = False
            return response

        if requested_goal is None:
            self.get_logger().warn(f'Incoming task rejected: Non-existent goal {request.goal}!')
            response.success = False
            return response

        if requested_agent.active:
            self.get_logger().warn(f'Incoming task rejected: Agent {request.agent} is currently active!')
            response.success = False
            return response

        if requested_goal.active:
            self.get_logger().warn(f'Incoming task rejected: Goal {request.goal} has already been set!')
            response.success = False
            return response
        
        # set to active for agents and goals
        requested_agent.active = True
        requested_goal.active = True

        # add ongoing task to track them
        self.task_count += 1
        new_task = TaskData(
            id=self.task_count,
            agent_id=requested_agent.id,
            goal_id=requested_goal.id
        )
        self.tasks.append(new_task)
        self.get_logger().info(f'Incoming task: Robot {request.agent} to Goal {request.goal}')

        response.success = True
        return response

    def send_path_length_callback(self, request: SendPathLength.Request, response: SendPathLength.Response):
        new_path_data = PathData(
            path_length = request.path_length,
            agent_id = request.agent,
            goal_id = request.goal
        )

        self.calculated_paths.append(new_path_data)
        self.get_logger().info(f'Incoming Path Data! New Path length of {new_path_data.path_length}')

        response.success = True
        return response

# ----------------------------------------------------------------------------

    def send_compute_request(self, agent_id: str, goal_id: str):
        """Send a compute path request to agent"""
        agent_compute_client = next((client for client in self.agent_compute_goal_clients if agent_id in client.srv_name), None)

        if agent_compute_client is None:
            print(f'{self.agent_compute_goal_clients}')
            self.get_logger().warn(f'Goal request failed: Non-existent compute_client for agent {agent_id}! Ignoring...')
            return

        while not agent_compute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{agent_compute_client.srv_name} service not available, waiting again...')

        marker_tf = self.get_marker_transform(int(goal_id))

        goal_req = SendGoal.Request()
        goal_req.x = marker_tf.transform.translation.x
        goal_req.y = marker_tf.transform.translation.y

        goal_req.goal_id = goal_id

        self.get_logger().info(f'Computing path from agent {agent_id} to goal {goal_id}...')
        future = agent_compute_client.call_async(goal_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_nav_request(self, agent_id: str, goal_id: str):
        """Send a start task request to agent"""
        agent_nav_client = next((client for client in self.agent_nav_goal_clients if agent_id in client.srv_name), None)

        if agent_nav_client is None:
            self.get_logger().warn(f'Goal request failed: Non-existent nav_client for agent {agent_id}! Ignoring...')
            return

        while not agent_nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{agent_nav_client.srv_name} service not available, waiting again...')

        marker_tf = self.get_marker_transform(int(goal_id))

        goal_req = SendGoal.Request()
        goal_req.x = marker_tf.transform.translation.x
        goal_req.y = marker_tf.transform.translation.y

        self.get_logger().info(f'Sending agent {agent_id} to goal {goal_id}')
        future = agent_nav_client.call_async(goal_req)
        return future.result()

# ----------------------------------------------------------------------------

    def get_marker_transform(self, index) -> TransformStamped:
        """Gets the transform of a given marker index"""
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
            return TransformStamped()

    def get_object_by_id(self, object_list: list[ObjectData], id: str) -> ObjectData | None:
        """Find and return an object in a list by it's ID"""
        return next((object for object in object_list if object.id == id), None)

    def send_agent_to_goal(self, task_agent_id: str, task_goal_id: str) -> None:
        """Function that sends agent to task"""

        if not task_agent_id or not task_goal_id:
            self.get_logger().warn('Missing agent or goal id, skipping request...')
            return

        self.send_nav_request(task_agent_id, task_goal_id)

        # set active to false
        # FIX: Need to make this work with actions so that it 
        # removes task when robot has actually completed it

# ----------------------------------------------------------------------------

    def loop(self) -> None:
        self.get_logger().info(f'Currently has: {len(self.tasks)} active tasks')

        complete_tasks: list[TaskData] = []
        free_agents = [agent for agent in self.agents if not agent.active]
        print(free_agents)
        for task in self.tasks:

            # check if it is single-robot (in this case, if the first in agents is equal to '')
            # if self.agents[0].id == task.agent_id or task.agent_id != '':
            #     self.send_agent_to_goal(task.agent_id, task.goal_id)

            if not task.calculated:
                for agent in free_agents:
                    self.send_compute_request(agent.id, task.goal_id)
                task.calculated = True
                continue

            # find the closest with the calculated stuff
            # TODO: Can try and make this like better by having the path data use a task id
            relevant_calculations = [pd for pd in self.calculated_paths if pd.goal_id == task.goal_id]

            shortest_distance = 100.0
            shortest_pd = None
            for pd in relevant_calculations:
                if pd.path_length < shortest_distance:
                    shortest_distance = pd.path_length
                    shortest_pd = pd

            if shortest_pd is None:
                self.get_logger().error('Could not find the closest agent! Skipping task...')
                continue

            closest_agent = self.get_object_by_id(self.agents, shortest_pd.agent_id)
            if closest_agent is None:
                self.get_logger().error('Could not find agent by id! Skipping task...')
                continue

            closest_agent.active = True
            self.send_agent_to_goal(shortest_pd.agent_id, shortest_pd.goal_id)
            complete_tasks.append(task)

        # FIX: Here would be where a service receives completion notices for all agents
        for task in complete_tasks:
            self.get_logger().info(f'Removing {task}')
            self.tasks.remove(task)
            


def main(args = None) -> None:
    rclpy.init(args = args)

    node = MissionNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
