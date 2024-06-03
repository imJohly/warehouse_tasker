from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')
    goal_count = LaunchConfiguration('goal_count', default=1)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace',
            default_value=namespace,
            description='Robot namespace.'
        ),
        DeclareLaunchArgument(
            name='goal_count',
            default_value=goal_count,
            description='Number of goals that the mission node needs to keep initialise and keep track of.'
        ),
        Node(
            namespace=namespace,
            package='warehouse_tasker',
            executable='old_agent_node',
            output='screen',
            parameters=[{
                'goal_count': goal_count,
            }],
            remappings=[('/tf', 'tf')]
        ),
    ])
