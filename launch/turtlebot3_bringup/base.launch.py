#!/usr/bin/env python3
#
# Copyright 2022 ROBOTIS CO., LTD.
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
# Author: Darby Lim

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Specifying namespace of turtlebot3'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether execute rviz2'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake command interfaces for sensors used for simple simulations. \
            Used only if "use_fake_hardware" parameter is true.'
        )
    )

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    remapping_tf = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]

    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('turtlebot3_description'),
                    'urdf',
                    'turtlebot3_' + os.environ['TURTLEBOT3_MODEL'] + '.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_commands,
        ]
    )

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('warehouse_tasker'),
            'params',
            # os.environ['TURTLEBOT3_MODEL'],
            namespace,
            'hardware_controller_manager.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('turtlebot3_bringup'),
            'rviz',
            'turtlebot3.rviz'
        ]
    )

    control_node = Node(
        namespace=namespace,
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': urdf_file},
            controller_manager_config
        ],
        remappings=[
            ('~/cmd_vel_unstamped', 'cmd_vel'),
            ('~/odom', 'odom')
        ] + remapping_tf,
        output="both",
        condition=UnlessCondition(use_sim))

    robot_state_pub_node = Node(
        namespace=namespace,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        remappings=remapping_tf,
        output='screen'
    )

    rviz_node = Node(
        namespace=namespace,
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    joint_state_broadcaster_spawner = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
        remappings=remapping_tf,
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '-c', 'controller_manager'],
        remappings=remapping_tf,
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    imu_broadcaster_spawner = Node(
        namespace=namespace,
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster'],
        remappings=remapping_tf,
        output='screen',
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        )

    delay_imu_broadcaster_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[imu_broadcaster_spawner],
            )
        )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
