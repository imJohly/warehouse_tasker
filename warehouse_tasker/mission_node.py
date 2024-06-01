import argparse
from dataclasses import asdict, dataclass

import rclpy
from rclpy.clock import Time
from rclpy.duration import Duration
from rclpy.node import Client, Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from warehouse_tasker_interfaces.srv import SendTask, Register, SendGoal

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
    def __init__(self, number_of_goals: int) -> None:
        super().__init__('mission_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.goals: list[ObjectData]    = []
        self.agents: list[ObjectData]   = []

        self.tasks: list[TaskData]      = []
        self.task_count: int            = 0

        # Services
        self.registration_service       = self.create_service(Register, 'register_agent', self.registration_callback)
        self.task_service               = self.create_service(SendTask, 'send_task', self.send_task_callback)

        # Clients
        self.agent_clients: list[Client] = []

        self.initialise_goals(number_of_goals)
        self.loop_timer = self.create_timer(1.0, self.loop)

# ----------------------------------------------------------------------------

    def send_task_callback(self, request: SendTask.Request, response: SendTask.Response):
        requested_agent = next((agent for agent in self.agents if agent.id == str(request.agent)), None)
        requested_goal = next((goal for goal in self.goals if goal.id == str(request.goal)), None)

        # TODO: need to change service to use a string type for namespaces
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
        self.agents.append(ObjectData(id=request.id, active=False))

        # create new client for registered agent
        # FIX: Needs to check if it is a valid namespace
        new_client = self.create_client(SendGoal, f'{request.id}/send_goal') 
        self.agent_clients.append(new_client)
        
        self.get_logger().info(f'Registered agent {request.id}')

        response.success = True
        return response

# ----------------------------------------------------------------------------

    def initialise_goals(self, number_of_goals: int) -> None:
        """Initialises goals"""
        for goal_index in range(number_of_goals):
            self.goals.append(ObjectData(id=str(goal_index), active=False))

        self.get_logger().info(f'Initialised {number_of_goals} goals.')

    # TODO: include agent client parameter
    def send_request(self, agent_id: str, goal_id: str):
        """Send a start task request to agent"""
        agent_client = next((client for client in self.agent_clients if agent_id in client.srv_name), None)

        if agent_client is None:
            self.get_logger().warn('Goal request failed: Non-existent agent! Ignoring...')
            return

        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        marker_tf = self.get_marker_transform(int(goal_id))

        goal_req = SendGoal.Request()
        goal_req.x = marker_tf.transform.translation.x
        goal_req.y = marker_tf.transform.translation.y

        self.get_logger().info(f'Sending agent to goal {goal_id}')
        future = agent_client.call_async(goal_req)
        return future.result()

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

    def loop(self) -> None:
        complete_tasks: list[TaskData] = []

        for task in self.tasks:
            self.send_request(task.agent_id, task.goal_id)

            task_agent = self.get_object_by_id(self.agents, task.agent_id)
            task_goal = self.get_object_by_id(self.goals, task.goal_id)
            if task_agent is None or task_goal is None:
                self.get_logger().warn('Could not set active task to false!')
                continue

            # reset active to false
            # TODO: Need to make this work with actions so that it 
            # removes task when robot has actually competed it
            task_agent.active = False
            task_goal.active = False

            # add tasks to completed list
            complete_tasks.append(task)

        for task in complete_tasks:
            self.get_logger().info(f'Removing {task}')
            self.tasks.remove(task)
            
        self.get_logger().info(f'Currently has: {len(self.tasks)} active tasks')


def main(args = None) -> None:
    rclpy.init(args = args)
    
    parser = argparse.ArgumentParser(description='Mission node arguments.')
    parser.add_argument('goals', metavar='-g', type=int, help='the number of goals to initialise')
    arguments = parser.parse_args()

    node = MissionNode(arguments.goals)
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
