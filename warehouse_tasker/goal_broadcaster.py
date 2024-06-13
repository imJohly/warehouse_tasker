import rclpy
from rclpy.node import Node
from rclpy.clock import Time

import yaml

from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from geometry_msgs.msg import TransformStamped

# TODO: Broadcast any goal Tfs to a marker/goal_visual topic

class GoalBroadcaster(Node):

    def __init__(self) -> None:
        super().__init__('goal_broadcaster')

        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter('marker_index', 0)
        self.declare_parameter('marker_yaml_path', '')

        marker_yaml_path = self.get_parameter('marker_yaml_path').get_parameter_value().string_value
        self.import_goals(marker_yaml_path)

        self.transfer_transforms()

    def import_goals(self, file_path: str):
        self.marker_locations = {}
        with open(file_path, 'r') as stream:
            self.marker_locations = yaml.safe_load(stream)
        self.get_logger().info('Imported marker locations.')

    def transfer_transforms(self):
        # try:
        index = self.get_parameter('marker_index').get_parameter_value().integer_value

        marker_transform = TransformStamped()
        marker_transform.header.frame_id = 'map'
        marker_transform.child_frame_id = f'marker{index}'

        x = self.marker_locations['positions'][index]['translation'][0]
        y = self.marker_locations['positions'][index]['translation'][1]
        z = self.marker_locations['positions'][index]['translation'][2]

        self.get_logger().info(f"{index}: {self.marker_locations['positions'][index]}")

        marker_transform.transform.translation.x = float(x)
        marker_transform.transform.translation.y = float(y)
        marker_transform.transform.translation.z = float(z)

        marker_transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(marker_transform)
        self.get_logger().info(f'Broadcasting marker{index} with translation {marker_transform.transform.translation}')

        # except BaseException as e:
        #     self.get_logger().warn(f"Error transferring transform: {e}")

def main():
    rclpy.init()
    
    node = DropZoneBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
