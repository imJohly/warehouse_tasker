import rclpy
from rclpy.node import Node

from warehouse_tasker_interfaces.srv import SendTask

class TaskReceiver(Node):
    def __init__(self) -> None:
        super().__init__('task_receiver_node')

        self.create_service(
            srv_type=SendTask,
            srv_name='send_task',
            callback=self.send_task_callback
        )

    def send_task_callback(self, request, response):
        self.robots_and_goals.append((request.robot, request.goal))
        self.get_logger().info(f'Incoming task: Robot {request.robot} to Goal {request.goal}')
        return response


def main(args = None) -> None:
    rclpy.init(args=args)
    node = TaskReceiver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
