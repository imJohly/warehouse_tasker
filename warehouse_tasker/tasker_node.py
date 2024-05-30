import rclpy
from rclpy.node import Node, Sequence, math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from typing import List

class PathSubscriber(Node):

    def __init__(self) -> None:
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
                msg_type=Path,
                topic='plan',
                callback=self.path_listener_callback,
                qos_profile=10)
        self.subscription

        print('initialised path counter!')

    def path_listener_callback(self, msg: Path) -> None:
        path_poses: Sequence[PoseStamped] = msg.poses

        total_distance: float = 0.0
        for i, pose in enumerate(path_poses):
            prev_pose = path_poses[i-1].pose.position
            curr_pose = pose.pose.position
            
            distance = math.sqrt(
                    (prev_pose.x - curr_pose.x)**2 +
                    (prev_pose.y - curr_pose.y)**2
            )

            total_distance += distance
        
        self.get_logger().info(f'Path has total distance of {total_distance:.2f}!')

def main(args: None = None):
    rclpy.init(args = args)

    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
