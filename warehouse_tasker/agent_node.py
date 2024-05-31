import argparse

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# from nav2_msgs.action import ComputePathToPose
from std_srvs.srv import SetBool

from warehouse_tasker_interfaces.srv import SendGoal, Register

from warehouse_tasker.agent_action_client import NavActionClient

class AgentNode(Node):

    def __init__(self, name) -> None:
        super().__init__('agent_node')

        # self._compute_action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.goal_service = self.create_service(SendGoal, 'send_goal', self.goal_service_callback)

        self.registration_client    = self.create_client(Register, 'register_agent')
        self.door_client            = self.create_client(SetBool, 'open_door')

        self.nav_goal_publisher = self.create_publisher(
            msg_type=PoseStamped,
            topic='goal_pose',
            qos_profile=5,
        )

        self.register_agent(name)
        self.create_timer(1.0, self.loop)

# -------------------------------------------------------------------------------------------

    def register_agent(self, name):
        while not self.registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        register_req = Register.Request()
        register_req.robot_name = name

        self.get_logger().info(f'Registering agent {name}')

        future = self.registration_client.call_async(register_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    def activate_payload_mechanism(self) -> bool:
        while not self.door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        payload_req = SetBool.Request()
        payload_req.data = True

        self.get_logger().info(f'Calling service to payload mechanism...')

        future = self.door_client.call_async(payload_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    # def send_compute_goal(self, goal: PoseStamped):
    #     goal_msg = ComputePathToPose.Goal()
    #     goal_msg.goal = goal
    #
    #     self._compute_action_client.wait_for_server()
    #
    #     self._send_compute_goal_future = self._compute_action_client.send_goal_async(goal_msg)
    #     self._send_compute_goal_future.add_done_callback(self.compute_goal_response_callback)
    #
    # def compute_goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return
    #
    #     self.get_logger().info('Goal accepted :)')
    #
    #     self._get_compute_result_future = goal_handle.get_result_async()
    #     self._get_compute_result_future.add_done_callback(self.get_compute_result_callback)
    #
    # def get_compute_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Result: {len(result.path.poses)}')
    #     self.get_logger().info(f'Path Length: {self.calculate_path_distance(result.path)}')
    #     
    #     time.sleep(0.5)
    #     return result

# -------------------------------------------------------------------------------------------

    def goal_service_callback(self, request, response):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.orientation.w = 1.0

        goal.pose.position.x = request.x
        goal.pose.position.y = request.y
        self.get_logger().info(f'Received goal locations - x: {request.x}, y: {request.y}')

        self.nav_goal_publisher.publish(goal)
        
        # rclpy.spin_once(self, timeout_sec=1.0)
        #TODO: add error handling
        
        response.success = True
        return response


# -------------------------------------------------------------------------------------------

    def calculate_path_distance(self, path: Path) -> float:
        """Calculates the length of a path."""
        path_distance: float = 0.0
        for i, pose in enumerate(path.poses):
            prev_pose = path.poses[i-1].pose.position
            curr_pose = pose.pose.position
            path_segment_distance = math.sqrt((prev_pose.x - curr_pose.x)**2 + (prev_pose.y - curr_pose.y)**2)
            path_distance += path_segment_distance

        return path_distance

# -------------------------------------------------------------------------------------------

    def start_nav_client(self, goal: PoseStamped):
        nav_client = NavActionClient()
        nav_client.send_nav_goal(goal)
        rclpy.spin(nav_client)

    def loop(self):
        print('looping')

def main(args = None) -> None:
    rclpy.init(args = args)

    parser = argparse.ArgumentParser(description='Mission node arguments.')
    parser.add_argument('name', help='namespace of robot')
    arguments = parser.parse_args()

    node = AgentNode(arguments.name)
    node.get_logger().info(f'Initialised Agent...')

    rclpy.spin(node)
    rclpy.shutdown()

    print('done!')

if __name__ == '__main__':
    main()
