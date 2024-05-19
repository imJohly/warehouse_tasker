from typing import List, Tuple
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped # Message type for transform
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

import math
import numpy as np

class VoronoiPatherNode(Node):

    def __init__(self):
        super().__init__('voronoi_pather')

        # Subscribes to the /scan topic for laser scan
        self.scan_subscriber = self.create_subscription(
            msg_type=LaserScan, 
            topic="scan", 
            callback=self.scan_callback,
            qos_profile=100
        )

        # Subscribes to the /odom topic for odometry
        self.odom_subscriber = self.create_subscription(
            msg_type=Odometry, 
            topic="odom", 
            callback=self.odom_callback,
            qos_profile=100
        )


        # Publisher for the path
        self.path_publisher = self.create_publisher(
            msg_type=Path,
            topic="my_path",
            qos_profile=10
        )

        self.scan = LaserScan()
        self.path = Path()

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def get_current_transform(self):
        pass

    def laser_scan_to_2d_position(self):
        scan_positions = []
        for scan_index, scan_range in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + scan_index * self.scan.angle_increment
            # if larger than range_max, skip
            if scan_range > self.scan.range_max:
                continue

            x = scan_range * math.cos(angle)
            y = scan_range * math.sin(angle)

            scan_positions.append((x, y))
        
        return scan_positions

    def create_path(self):
        # get the average of all the points
        # and place a point/add to path

        clock = self.get_clock()

        points = np.array(self.laser_scan_to_2d_position())

        # check for empty points array
        if not points.any():
            return None

        average_pos_2d = np.mean(points, axis=0)

        average_pose = PoseStamped()
        average_pose.header.stamp = clock.now().to_msg()
        average_pose.header.frame_id = 'map'
        average_pose.pose.position.x = average_pos_2d[0] + self.odom.pose.pose.position.x
        average_pose.pose.position.y = average_pos_2d[1] + self.odom.pose.pose.position.y
        average_pose.pose.position.z = 0.5

        # print(average_pose.pose.position.x, average_pose.pose.position.y)
        self.path.header.stamp = clock.now().to_msg()
        self.path.header.frame_id = 'map'
        self.path.poses.append(average_pose)

        self.path_publisher.publish(self.path)


def main(args = None) -> None:
    rclpy.init(args = args)

    print('initialising pather2')
    scan_node = VoronoiPatherNode()
    while rclpy.ok():
        # Do stuff

        path = scan_node.create_path()

        rclpy.spin_once(scan_node, timeout_sec=0.5)

if __name__ == '__main__':
    main()
