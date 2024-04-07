import rclpy
from rclpy.node import Node
from rclpy.clock import Time
from rclpy.duration import Duration

import argparse

from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from geometry_msgs.msg import TransformStamped

class MarkerStatic(Node):

    def __init__(self) -> None:
        super().__init__('node')

        self.declare_parameter('marker_name', 'marker_0')

        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

    def transfer_transforms(self):
        marker_name = self.get_parameter('marker_name').get_parameter_value().string_value
        try:
            # Get available transforms from /tf
            marker_transform: TransformStamped = self.tf_buffer.lookup_transform(
                source_frame=marker_name,
                target_frame='map',
                time=Time(seconds=0)
            )
            self.static_tf_broadcaster.sendTransform(marker_transform)
            self.get_logger().info(f'Updated transform for {marker_name}')

        except BaseException as e:
            self.get_logger().warn(f"Error transferring transform: {e}")

        rclpy.spin_once(self)
 

def main():
    rclpy.init()

    node = MarkerStatic()
    while rclpy.ok():
        node.transfer_transforms()
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    main()
