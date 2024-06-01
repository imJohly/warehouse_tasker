import argparse
from dataclasses import dataclass

import rclpy
from rclpy.clock import Time
from rclpy.duration import Duration
from rclpy.node import Client, Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from warehouse_tasker_interfaces.srv import SendTask, SendTaskAny, Register, SendGoal

@dataclass
class ObjectData:
    id:         str
    active:     bool = False

@dataclass
class TaskData:
    id:         int
    agent_id:   str
    goal_id:    str

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

        # Services
        self.registration_service       = self.create_service(Register, 'register_agent', self.registration_callback)
        self.task_service               = self.create_service(SendTask, 'send_task', self.send_task_callback)
        self.task_any_service           = self.create_service(SendTaskAny, 'send_task_any', self.send_task_any_callback)

        # Clients
        self.agent_compute_goal_clients: list [Client]  = []
        self.agent_nav_goal_clients: list[Client]       = []

        # Transform buffer and listener to get marker tfs
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.initialise_goals(self.get_parameter('goal_count').get_parameter_value().integer_value)
        self.loop_timer = self.create_timer(1.0, self.loop)

# ----------------------------------------------------------------------------

    def initialise_goals(self, number_of_goals: int) -> None:
        """Initialises goals"""
        for goal_index in range(number_of_goals):
            self.goals.append(ObjectData(id=str(goal_index)))

        self.get_logger().info(f'Initialised {number_of_goals} goals.')

# ----------------------------------------------------------------------------

    # FIX: Needs to account for multi-robot tasks,
    # currently works for sending goals to specific agents.
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
            goal_id=requested_goal.id
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

        self.get_logger().info(f'Computing path from agent {agent_id} to goal {goal_id}...')
        future = agent_compute_client.call_async(goal_req)
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

        self.get_logger().info(f'Sending agent to goal {goal_id}')
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
            self.get_logger().info(f'Successfully found tf, {transform.transform.translation}')
            return transform

        except BaseException as e:
            self.get_logger().warn(f'Error getting transform: {e}')
            return TransformStamped()

    def get_object_by_id(self, object_list: list[ObjectData], id: str) -> ObjectData | None:
        """Find and return an object in a list by it's ID"""
        return next((object for object in object_list if object.id == id), None)

    def send_agent_to_goal(self, task_agent_id: str, task_goal_id: str) -> None:
        """Function that sends agent to task"""
        self.send_nav_request(task_agent_id, task_goal_id)

        # set active to false
        # FIX: Need to make this work with actions so that it 
        # removes task when robot has actually completed it
        task_agent = self.get_object_by_id(self.agents, task_agent_id)
        task_goal = self.get_object_by_id(self.goals, task_goal_id)
        if task_agent is None or task_goal is None:
            self.get_logger().warn('Could not set active task to false!')
            return
        task_agent.active = False
        task_goal.active = False


    def loop(self) -> None:
        complete_tasks: list[TaskData] = []

        free_agents = [agent for agent in self.agents if not agent.active]

        for task in self.tasks:
            # check if it is single-robot (in this case, if the first in agents is equal to '')
            # if self.agents[0].id == task.agent_id or task.agent_id != '':
            #     self.send_agent_to_goal(task.agent_id, task.goal_id)

            # else, multi-robot (1: calculate all distances)
            agents_distance_to_goal: list[float] = []
            for agent in free_agents:
                result = self.send_compute_request(agent.id, task.goal_id)
                self.get_logger().info(f'Result: {result}')
                # agents_distance_to_goal.append()

            # add tasks to completed list
            complete_tasks.append(task)

        # FIX: Here would be where a service receives completion notices for all agents
        for task in complete_tasks:
            self.get_logger().info(f'Removing {task}')
            self.tasks.remove(task)
            
        self.get_logger().info(f'Currently has: {len(self.tasks)} active tasks')


def main(args = None) -> None:
    rclpy.init(args = args)

    node = MissionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
