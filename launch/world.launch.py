import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Get the launch directory
    warehouse_pkg_dir = get_package_share_directory('warehouse_tasker')
    warehouse_launch_file_dir = os.path.join(warehouse_pkg_dir, 'launch')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    namespace = LaunchConfiguration('namespace', default='turtleboi')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
        
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(warehouse_pkg_dir, 'worlds', 'warehouse_world.world'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    start_gazebo_client_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(warehouse_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'namespace'     : namespace,
            'x_pose'        : x_pose,
            'y_pose'        : y_pose
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
