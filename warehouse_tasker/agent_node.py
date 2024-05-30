import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose

from warehouse_tasker_interfaces.srv import SendGoal, Register

class AgentNode(Node):

    def __init__(self, name: int) -> None:
        super().__init__('agent_node')

        self.current_path = Path().poses

        self.distance_to_goal = 0

        self._compute_action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._nav_action_client     = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_service = self.create_service(SendGoal, 'send_goal', self.goal_service_callback)

        self.registration_client = self.create_client(Register, 'register_agent')
        while not self.registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        register_req = Register.Request()
        register_req.robot_name = name
        self.get_logger().info(f'Registering agent {name}')
        self.future = self.registration_client.call_async(register_req)
        rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

    def send_compute_goal(self, goal: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal

        self._compute_action_client.wait_for_server()

        self._send_goal_future = self._compute_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.compute_goal_response_callback)

    def compute_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_compute_result_callback)

    def get_compute_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {len(result.path.poses)}')
        self.get_logger().info(f'Path Length: {self.calculate_path_distance(result.path)}')
        # rclpy.shutdown()

    def send_nav_goal(self, goal: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._nav_action_client.wait_for_server()

        self._send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_nav_result_callback)

    def get_nav_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        # rclpy.shutdown()

# -------------------------------------------------------------------------------------------

    def goal_service_callback(self, request, response):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.orientation.w = 1.0

        goal.pose.position.x = request.x
        goal.pose.position.y = request.y

        self.send_nav_goal(goal)

        #TODO: add error handling
        
        response.success = True
        return response

# -------------------------------------------------------------------------------------------

    def activate_payload_mechanism(self) -> bool:
        success: bool = False


        return success

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

def main(args = None) -> None:
    rclpy.init(args = args)

    parser = argparse.ArgumentParser(description='Mission node arguments.')
    parser.add_argument('name', help='namespace of robot')
    arguments = parser.parse_args()

    node = AgentNode(arguments.name)
    node.get_logger().info(f'Initialised Agent...')

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
