#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': 0.01},
        {'name': 'tb2', 'x_pose': '1.4', 'y_pose': '0.0', 'z_pose': 0.01},
    ]

    TURTLEBOT3_MODEL = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )
    

    
#    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    package_dir = get_package_share_directory('warehouse_tasker')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        package_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(get_package_share_directory('warehouse_tasker'), 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
 
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('warehouse_tasker'), 'map', 'map_1715820556.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # last_action = None

    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items(),
                                    )

        # if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
        ld.add_action(bringup_cmd)

        # Save last instance for next RegisterEventHandler
        # last_action = bringup_cmd
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        # post_spawn_event = RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=last_action,
        #         # on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
        #         on_exit=[rviz_cmd, drive_turtlebot3_burger],
        #     )
        # )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        ld.add_action(initial_pose_cmd)
        ld.add_action(rviz_cmd)
        ld.add_action(drive_turtlebot3_burger)
        # ld.add_action(post_spawn_event)
        ld.add_action(declare_params_file_cmd)
    ######################

    return ld
