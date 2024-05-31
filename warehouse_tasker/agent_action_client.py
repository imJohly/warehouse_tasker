import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose


class NavActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')

        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_nav_goal(self, goal: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._nav_action_client.wait_for_server()

        self.get_logger().info('Sent Nav Goal...')

        self._send_nav_goal_future = self._nav_action_client.send_goal_async(goal_msg) # , feedback_callback=self.feedback_callback)
        self._send_nav_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.get_nav_result_callback)

    def get_nav_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.distance_remaining}')

# def main(args=None):
#     rclpy.init(args=args)
#
#     action_client = NavActionClient()
#
#     # action_client.send_nav_goal()
#
#     rclpy.spin(action_client)
#
# if __name__ == '__main__':
#     main()
