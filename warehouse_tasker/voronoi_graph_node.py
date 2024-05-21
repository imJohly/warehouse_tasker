from typing import List
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

import cv2
import math
import numpy as np

import threading

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

        self.scan_mutex_ = threading.Lock()
        self.map_mutex_ = threading.Lock()

        self.scan_ = LaserScan()
        self.map_ = Odometry()
        self.map_params_ = None

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

        # debugging purposes only
        # cv2.imwrite('import.jpg', image)


        self.map_mutex_.release()

    def update_map(self):
        self.map_mutex_.acquire(blocking=True)

        # create a graph from /map topic
        # g = VoronoiGrapher(map_data=self.map_)
        # graph = g.get_graph()

        # publish graph
        # grid = OccupancyGrid()
        # grid.header.stamp = self.get_clock().now()
        # grid.data = graph
        # self.voronoi_publisher.publish()

        # voronoi = 

        grapher = VoronoiGrapher(self.map_)
        map = grapher.process_map()
        cv2.imwrite('processed.jpg', map)

        self.map_mutex_.release()

def main(args = None) -> None:
    rclpy.init(args = args)

    print('initialising voronoi_node')
    scan_node = VoronoiPatherNode()
    while rclpy.ok():
        # Do stuff

        path = scan_node.update_map()

        rclpy.spin_once(scan_node, timeout_sec=5)

if __name__ == '__main__':
    main()
