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
        'marker_locations.yaml'
    )

    config = {}
    with open(marker_config, 'r') as stream:
        config = yaml.safe_load(stream)
    number_of_markers = len(list(config['positions']))

    marker_nodes = []
    for i in range(number_of_markers):
        marker_nodes.append(
            Node(
                package='warehouse_tasker',
                executable='drop_zone_broadcaster',
                output='screen',
                parameters=[{'marker_yaml_path': marker_config,
                             'marker_index': i}],
                remappings=[('/tf_static', '/tb1/tf_static')]
            )
        )

    marker_nodes = []
    for i in range(number_of_markers):
        marker_nodes.append(
            Node(
                package='warehouse_tasker',
                executable='drop_zone_broadcaster',
                output='screen',
                parameters=[{'marker_yaml_path': marker_config,
                             'marker_index': i}]
            )
        )

    return LaunchDescription([
    ] + marker_nodes)
