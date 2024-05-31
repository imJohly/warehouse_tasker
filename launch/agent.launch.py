import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('warehouse_tasker')

    namespace = LaunchConfiguration('namespace', default='')
    declare_namespace = DeclareLaunchArgument(
        name='namespace',
        default_value=namespace,
        description='robot namespace'
    )

    agent_node = Node(
        namespace=namespace,
        package='warehouse_tasker',
        executable='agent_node',
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        agent_node
    ])
