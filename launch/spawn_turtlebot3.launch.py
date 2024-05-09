import os
from typing import DefaultDict

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    namespace   = LaunchConfiguration('namespace', default='tb1')
    x_pose      = LaunchConfiguration('x_pose', default='0.0')
    y_pose      = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value = namespace,
        description='Set robot namespace')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value=x_pose,
        description='Set x position')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value=y_pose,
        description='Set x position')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',          namespace,
            '-robot_namespace', namespace,
            '-file',            urdf_path,
            '-x',               x_pose,
            '-y',               y_pose,
            '-z',               '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
