import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
import yaml

from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray

class GoalBroadcaster(Node):

    def __init__(self) -> None:
        super().__init__('goal_broadcaster')

        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.goal_marker_publisher = self.create_publisher(MarkerArray, 'goal_markers', 5)

        self.declare_parameter('marker_yaml_path', '')
        self.declare_parameter('publish_visual', True)

        marker_yaml_path = self.get_parameter('marker_yaml_path').get_parameter_value().string_value

        self.marker_locations = self.import_goals(marker_yaml_path)
        self.transfer_transforms()
        self.publish_visual_markers()

    def import_goals(self, file_path: str) -> dict:
        """Imports goal locations from a YAML file.

        Arguments:
            file_path - A string path to the goal locations YAML file

        Returns:
            A dictionary containing imported goal locations.
        """
        marker_locations = {}
        with open(file_path, 'r') as stream:
            marker_locations = yaml.safe_load(stream)
        self.get_logger().info('Imported marker locations.')

        return marker_locations

    def transfer_transforms(self) -> None:
        """Gets all marker transforms and publishes them as static tfs."""
        marker_transforms = []
        for i in self.marker_locations['positions']:
            marker_transform = TransformStamped()
            try:
                marker_transform.header.frame_id = 'map'
                marker_transform.child_frame_id = f'marker{i}'

                x = self.marker_locations['positions'][i]['translation'][0]
                y = self.marker_locations['positions'][i]['translation'][1]
                z = self.marker_locations['positions'][i]['translation'][2]

                marker_transform.transform.translation = Vector3(x=float(x), y=float(y), z=float(z))

                marker_transform.transform.rotation.w = 1.0
                marker_transforms.append(marker_transform)
                self.get_logger().info(f'Broadcasting marker{i} with translation: {self.marker_locations["positions"][i]}')

            except BaseException as e:
                self.get_logger().warn(f'Error transferring transform: {e}')

        self.static_tf_broadcaster.sendTransform(marker_transforms)

    def publish_visual_markers(self) -> None:
        """Gets all marker positions and publishes them as a visual marker array."""
        array_msg = MarkerArray()

        for i in self.marker_locations['positions']:
            position = self.marker_locations['positions'][i]['translation']
            marker_msg = Marker()
            marker_msg.header.frame_id = 'map'
            marker_msg.type = Marker.ARROW
            marker_msg.id = i
            marker_msg.action = Marker.ADD

            marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2]+0.6)
            marker_msg.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
            marker_msg.scale = Vector3(x=0.5, y=0.1, z=0.1)
            marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.5)

            array_msg.markers.append(marker_msg)
            self.get_logger().info(f'Set up marker {i}')

        self.goal_marker_publisher.publish(array_msg)
        self.get_logger().info('Published marker array!')

def main():
    rclpy.init()
    
    node = GoalBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
