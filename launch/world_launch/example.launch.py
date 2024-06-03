import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # DECLARE PARAMETER THAT YOU CAN CHANGE WHEN LAUNCHING
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=use_sim_time,
        description='Use simulation timing'
    )

    # DECLARE 'LAUNCH DESCRIPTION'
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_cartographer'),
            'launch',
            'cartographer.launch.py'
        )),
        launch_arguments={'use_sim': use_sim_time,
                          'start_rviz' : 'false'}.items()
    )

    # ACTUALLY DO THOSE THINGS
    return LaunchDescription([ 
        declare_use_sim_time,
        cartographer_launch,
    ])
