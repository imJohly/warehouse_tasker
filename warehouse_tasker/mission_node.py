import argparse
from concurrent import futures
from time import time

from numpy import append, number
import rclpy
from rclpy.clock import Time
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Transform, TransformStamped  # Message type for transform

from warehouse_tasker_interfaces.srv import SendTask, Register, SendGoal

from warehouse_tasker import agent_action_client

class MissionNode(Node):
    def __init__(self, number_of_goals: int) -> None:
        super().__init__('mission_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

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

        self.agent_goal_client = self.create_client(SendGoal, 'send_goal')

        self.loop_timer = self.create_timer(1.0, self.loop)

    def send_task_callback(self, request, response):
        if not request.goal in self.goal_states:
            self.get_logger().info(f'Incoming task failed: Non-existent Goal {request.goal}')
            response.success = False
            return response

        if not f'{request.robot}' in self.agents:
            self.get_logger().info(f'Incoming task failed: Non-existent Agent {request.robot}')
            response.success = False
            return response

        # check if the goal is unset,
        if self.goal_states[request.goal] == 'set':
            self.get_logger().info(f'Incoming task: Failed to set Robot {request.robot} to Goal {request.goal}')
            response.success = False
            return response

        self.get_logger().info(f'Incoming task: Robot {request.robot} to Goal {request.goal}')

        self.robots_and_goals.append((request.robot, request.goal))
        self.goal_states[request.goal] = 'set'
        print(self.send_request(request.goal))

        response.success = True
        return response

    def registration_callback(self, request, response):
        self.get_logger().info(f'Registering agent {request.robot_name}')
        self.agents.append(request.robot_name)

        #TODO: add error handling

        response.success = True
        return response

    def get_marker_transform(self, index):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame=f'marker{index}',
                    time=Time(seconds=0),
                    timeout=Duration(seconds=0.1),
                    )
            self.get_logger().info(f'Successfully found tf, {transform.transform.translation}')
            return transform

        except BaseException as e:
            self.get_logger().warn(f'Error getting transform: {e}')
            return TransformStamped()

    def send_request(self, goal_index: int):
        while not self.agent_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        goal_req = SendGoal.Request()
        goal_req.x = 0.0
        goal_req.y = 0.0
        self.get_logger().info(f'Sending agent to goal {goal_index}')

        future = self.agent_goal_client.call_async(goal_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def loop(self) -> None:
        if self.number_of_goals <= 0:
            self.get_logger().error(f'No goals initialised, shutting down...')
            self.destroy_node()
            return

        self.get_logger().info(f'Currently has: {len(self.robots_and_goals)} goals')
        
        # for rag in self.robots_and_goals:
        #     # add namespace here
        #     # agent_goal_client = self.create_client(
        #     #     srv_type=SendGoal,
        #     #     srv_name='send_goal',
        #     # )
        #
        #     marker_tf = self.get_marker_transform(rag[1])
        #
        #     goal_req = SendGoal.Request()
        #     goal_req.x = marker_tf.transform.translation.x
        #     goal_req.y = marker_tf.transform.translation.y
        #     self.get_logger().info(f'Sending agent to goal {rag[1]}')
        #     future = self.agent_goal_client.call_async(goal_req)
        #     rclpy.spin_until_future_complete(self, future)
        #
        #     # remove the goal task that was just complete
        #     self.robots_and_goals.pop(0)



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
