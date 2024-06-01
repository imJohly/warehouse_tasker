import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('warehouse_tasker')

    marker_yaml_path = LaunchConfiguration(
        'marker_yaml_path',
        default=os.path.join(
            package_dir,
            'params',
            'sim_marker_locations.yaml'))
    declare_marker_yaml_path = DeclareLaunchArgument(
        name='marker_yaml_path',
        default_value=marker_yaml_path,
        description='Marker Locations yaml file path'
    )

    marker_nodes = []

    for i in range(16):
        marker_nodes.append(
            Node(
                package='warehouse_tasker',
                executable='drop_zone_broadcaster',
                output='screen',
                parameters=[{'marker_yaml_path': marker_yaml_path,
                             'marker_index': i}]
            )
        )

    return LaunchDescription([
        declare_marker_yaml_path,
    ] + marker_nodes)
