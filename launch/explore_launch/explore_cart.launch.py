import os
import launch_ros


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declared_arguments = [
    #     DeclareLaunchArgument(
    #         "is_sim",
    #         default_value="true",
    #     )]

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=use_sim_time,
        description='Use simulation timing'
    )

    # package_name = 'auto_mapper'

    # auto_mapper = Node(
    #     package=package_name,
    #     executable="auto_mapper",
    #     name="auto_mapper",
    # )


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 
            'launch',
            'navigation2.launch.py'

            )
        )
    )

    cart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_cartographer'), 
            'launch',
            'cartographer.launch.py'

            )
        )
        # launch_arguments={'use_sim_time': is_sim,}.items()
    )




    return LaunchDescription([declare_use_sim_time,
                             nav2_launch,
                             cart_launch,
                             ])







# def create_slam_toolbox_node() -> object:
#     launch_file_path = PathJoinSubstitution(
#         [FindPackageShare(package_name), 'launch', 'online_async_launch.py'])
#     params_file = PathJoinSubstitution(
#         [FindPackageShare(package_name), "config", "mapper_params_online_async.yaml"])
#     return IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(launch_file_path),
#         launch_arguments={'use_sim_time': is_sim, 'slam_params_file': params_file, 'map_file_name': map_path}.items()
#     )

