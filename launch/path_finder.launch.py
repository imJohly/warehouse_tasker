import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node (
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='navigation_launch',
            output='screen'
            )
        ])
