import rclpy
from rclpy.node import Node
from rclpy.clock import Time
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped  # Message type for transform

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        # Specify source and target frames (replace with your frame names)
        self.source_frame = 'base_link'
        self.target_frame = 'map'

    def get_transform(self):
        try:
            # Get the transform at a specific time (optional, defaults to now)
            # transform: TransformStamped = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, now, timeout=Duration(seconds=5))
            # self.get_logger().info(f'Transform from {self.source_frame} to {self.target_frame}:\n{transform}')
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame=self.target_frame,
                    source_frame=self.source_frame,
                    time=Time(seconds=0),
                    timeout=Duration(seconds=0.1),
                    )
            self.get_logger().info(f'{transform.transform.translation}')

        except BaseException as e:
            self.get_logger().warn(f'Error getting transform: {e}')


def main(args = None) -> None:
    rclpy.init(args = args)

    print('initialising pather2')
    my_node = MyNode()
    while rclpy.ok():
        my_node.get_transform()
        rclpy.spin_once(my_node, timeout_sec=0.5)

if __name__ == '__main__':
    main()
