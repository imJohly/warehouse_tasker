from typing import List
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import MapMetaData, Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker

import cv2
import math
import numpy as np

import threading
import time

from warehouse_tasker.voronoi_grapher import VoronoiGrapher

# import custom modules
# from voronoi_grapher import VoronoiGrapher

def map_range(value, in_min, in_max, out_min, out_max):
  """
  Maps a value from one range to another.

  Args:
      value (float): The value to be mapped.
      in_min (float): The minimum value in the input range.
      in_max (float): The maximum value in the input range.
      out_min (float): The minimum value in the output range.
      out_max (float): The maximum value in the output range.

  Returns:
      float: The mapped value in the output range.
  """

  # Check for zero division
  if in_max - in_min == 0:
    return out_min  # Avoid division by zero

  # Normalize the value to the input range (0 to 1)
  normalized_value = (value - in_min) / (in_max - in_min)

  # Re-scale to the output range
  mapped_value = normalized_value * (out_max - out_min) + out_min

  return mapped_value

class VoronoiPatherNode(Node):

    def __init__(self):
        super().__init__('voronoi_pather')

        # Subscribes to the /odom topic for odometry
        self.map_subscriber = self.create_subscription(
            msg_type=OccupancyGrid, 
            topic="map", 
            callback=self.map_callback,
            qos_profile=5
        )

        self.voronoi_publisher = self.create_publisher(
            msg_type=OccupancyGrid,
            topic="voronoi_map",
            qos_profile=5
        )

        self.marker_publisher = self.create_publisher(
            msg_type=MarkerArray,
            topic='/grapher_nodes',
            qos_profile=100
        )

        self.scan_mutex_ = threading.Lock()
        self.map_mutex_ = threading.Lock()

        self.scan_ = LaserScan()
        self.map_ = []
        self.map_params_: MapMetaData = MapMetaData()

        self.nodes = []

        self.create_timer(5, self.update_map)
        self.create_timer(5, self.update_markers)

    def scan_callback(self, msg: LaserScan):
        self.scan_mutex_.acquire(blocking=True)
        self.scan_ = msg
        self.scan_mutex_.release()

    def map_callback(self, msg: OccupancyGrid):
        self.map_mutex_.acquire(blocking=True)

        self.map_params_ = msg.info

        # convert map msg into a cv2 compatible image
        map_data = msg.data
        height = self.map_params_.height
        width = self.map_params_.width
 
        image = np.zeros((height, width), dtype=np.uint8)

        for i in range(height):
            for j in range(width):
                index = i * width + j
                value = map_data[index]

                image[i][j] = map_range(value, 0, 100, 255, 0)
                if value == -1:
                    image[i][j] = 0
                elif value == 0:
                    image[i][j] = 255

        self.map_ = image
        
        self.get_logger().info('Found new map!')

        # debugging purposes only
        # cv2.imwrite('import.jpg', image)

        self.map_mutex_.release()

    def nodes_to_real(self, nodes) -> NDArray:
        """Convert list of nodes to real space"""
        real = np.array(nodes) * self.map_params_.resolution
        origin = self.map_params_.origin.position
        return [[node[0] + origin.x, node[1] + origin.y] for node in real]

    def update_map(self):
        if len(self.map_) == 0:
            print('finding map...')
            return

        self.map_mutex_.acquire(blocking=True)
        grapher = VoronoiGrapher(self.map_)
        # # Debugging images only
        # cv2.imwrite('processed.jpg', grapher.processed)
        # print('processed!')
        self.map_mutex_.release()

        self.nodes = self.nodes_to_real(grapher.nodes)

        self.get_logger().info('Updated nodes.')

    def update_markers(self):
        if not self.nodes:
            return

        markers = []
        for i, node in enumerate(self.nodes):
            markers.append(self.create_sphere_markers(id=i, x=node[0], y=node[1], z=0.1))

        msg = MarkerArray()
        msg.markers = markers
        self.marker_publisher.publish(msg)

        self.get_logger().info(f'published {len(self.nodes)} markers.')

    def create_sphere_markers(self, id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.id = id
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        
        return marker

def main(args = None) -> None:
    rclpy.init(args = args)

    print('initialising voronoi_node')
    scan_node = VoronoiPatherNode()
    rclpy.spin(scan_node)

if __name__ == '__main__':
    main()
