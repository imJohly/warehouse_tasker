import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

import math

class DistanceProcessing(Node):

    def __init__(self) -> None:
        super().__init__('distance_processor')
        self.subscription = self.create_subscription(
            msg_type=Path,
            topic='plan',
            callback=self.path_listener_callback,
            qos_profile=10
        )

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

    def distance_polling(self, robots) -> List[float]:
        # 1. query for distances
        


        # 2.

def main(args = None) -> None:
    rclpy.init(args = args)

    distance_processing = DistanceProcessing()
    while rclpy.ok():
        # my_node.get_transform()
        rclpy.spin_once(distance_processing, timeout_sec=0.5)

if __name__ == '__main__':
    main()
