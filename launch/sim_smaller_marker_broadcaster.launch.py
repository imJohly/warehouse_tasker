import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('warehouse_tasker')

    marker_config = os.path.join(
        package_dir,
        'params',
        'sim_smaller_marker_locations.yaml'
    )

    broadcaster_node = Node(
        package='warehouse_tasker',
        executable='goal_broadcaster',
        output='screen',
        parameters=[{'marker_yaml_path': marker_config}]
    )

    return LaunchDescription([
        broadcaster_node
    ])
