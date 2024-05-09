#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription, descriptions
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions.node import add_node_name

def generate_launch_description():
    ld = LaunchDescription()

    robots = [
        {'name':'tb1','x_pose':'-1.5','y_pose':'-0.5','z_pose':0.01},
        {'name':'tb2','x_pose':'-1.5','y_pose':'0.5','z_pose':0.01},
        # {'name':'tb3','x_pose':'1.5','y_pose':'-0.5','z_pose':0.01},
        # {'name':'tb4','x_pose':'1.5','y_pose':'0.5','z_pose':0.01},
    ]

    # Turtlebot model can be burger, waffle & waffle_pi
    TURTLEBOT3_MODEL = 'waffle_pi'

    # package paths
    pkg_dir                     = get_package_share_path('warehouse_tasker')
    nav_launch_dir              = os.path.join(pkg_dir, 'launch', 'nav2_bringup')
    turtlebot3_gazebo_pkg_dir   = get_package_share_path('turtlebot3_gazebo')

    # variables
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default = 'true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        turtlebot3_gazebo_pkg_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    # launch gazebo

    world = os.path.join(pkg_dir, 'worlds', 'warehouse_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    # load parameter files
    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # load singular map server
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    map_server=Node(
        package             ='nav2_map_server',
        executable          ='map_server',
        name                ='map_server',
        output              ='screen',
        parameters          =[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),},],
        remappings          =remappings)

    map_server_lifecyle=Node(
        package             ='nav2_lifecycle_manager',
        executable          ='lifecycle_manager',
        name                ='lifecycle_manager_map_server',
        output              ='screen',
        parameters          =[{'use_sim_time': use_sim_time},
                              {'autostart': True},
                              {'node_names': ['map_server']}])

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    

    ###

    last_action = None

    for robot in robots:
        namespace = ['/' + robot['name']]

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package     ='robot_state_publisher',
            namespace   =namespace,
            executable  ='robot_state_publisher',
            output      ='screen',
            parameters  =[{'use_sim_time': use_sim_time,
                           'publish_frequency': 10.0}],
            remappings  =remappings,
            arguments   =[urdf],
        )

        # Spawn call
        spawn_turtlebot3 = Node(
            package     ='gazebo_ros',
            executable  ='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_gazebo_pkg_dir,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3,
                            turtlebot_state_publisher],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3

    return ld


