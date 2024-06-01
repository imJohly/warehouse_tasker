import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory('warehouse_tasker')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=use_sim_time,
        description='Use simulation timing'
    )

    package_launch_dir = os.path.join(
        package_dir,
        'launch'
    )

    # world_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(package_launch_dir, 'world.launch.py')),
    # )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_cartographer'),
            'launch',
            'cartographer.launch.py'
        )),
        launch_arguments={'use_sim': use_sim_time,
                          'start_rviz' : 'false'}.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'launch',
            'navigation2.launch.py'
        )),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    marker_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_launch_dir,
            'sim_marker_broadcaster.launch.py'
        ))
    )

    mission_node = Node(
        package='warehouse_tasker',
        executable='mission_node',
        output='screen',
        arguments=['16']
    )

    agent_node = Node(
        package='warehouse_tasker',
        executable='agent_node',
        output='screen',
        arguments=['1']
    )

    post_mission_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_node,
            on_exit=[agent_node]
        )
    )

    return LaunchDescription([ 
        declare_use_sim_time,
        # world_launch,
        cartographer_launch,
        navigation_launch,
        marker_broadcaster_launch,
        mission_node,
        post_mission_launch,
    ])
