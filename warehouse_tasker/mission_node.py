import argparse

from numpy import append, number
import rclpy
from rclpy.node import Node

from warehouse_tasker_interfaces.srv import SendTask, Register

class MissionNode(Node):
    def __init__(self, number_of_goals: int) -> None:
        super().__init__('mission_node')
 
        self.robots_and_goals = []

        # initialise goal_states
        self.number_of_goals = number_of_goals
        self.goal_states = {}
        for goal in range(number_of_goals):
            self.goal_states[goal] = 'unset'
        self.get_logger().info(f'There are {number_of_goals} goals.')

        self.task_service = self.create_service(
            srv_type=SendTask,
            srv_name='send_task',
            callback=self.send_task_callback
        )

        self.agents = []
        self.registration_service = self.create_service(
            srv_type=Register,
            srv_name='register_agent',
            callback=self.registration_callback
        )

        self.loop_timer = self.create_timer(timer_period_sec=1, callback=self.loop)

    def send_task_callback(self, request, response):
        if not request.goal in self.goal_states:
            self.get_logger().info(f'Incoming task: Failed to set Robot {request.robot} to Goal {request.goal}')
            response.success = False
            return response

        # check if the goal is unset,
        if self.goal_states[request.goal] == 'set':
            self.get_logger().info(f'Incoming task: Failed to set Robot {request.robot} to Goal {request.goal}')
            response.success = False
            return response

        self.robots_and_goals.append((request.robot, request.goal))
        self.get_logger().info(f'Incoming task: Robot {request.robot} to Goal {request.goal}')

        self.goal_states[request.goal] = 'set'

        # start agent on new task
        # for one robot
        # self.agents

        response.success = True
        return response

    def registration_callback(self, request, response):
        self.get_logger().info(f'Registering agent {request.robot_name}')
        self.agents.append(request.robot_name)

        #TODO: add error handling

        response.success = True
        return response

    def loop(self) -> None:
        if self.number_of_goals <= 0:
            self.get_logger().error(f'No goals initialised, shutting down...')
            self.destroy_node()
            return

        self.get_logger().info(f'Currently has: {len(self.robots_and_goals)} goals')

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
