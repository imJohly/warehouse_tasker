import rclpy
from rclpy import Node
from rclpy.executors import MultiThreadedExecutor

class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__('agent_node')

    def main_loop(self) -> None:

        # TODO: Add a metric of some sort here later...
        self.get_logger().info('Looping...')

def main(args=None) -> None:
    rclpy.init(args=args)

    node = AgentNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

