from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

def generate_launch_description():
    # marker_1_node = Node(
    #     package='warehouse_tasker',
    #     executable='marker_mapper_node',
    #     output='screen',
    #     parameters=[{'marker_name': 'marker_1'}]
    # )
    #
    # ld = LaunchDescription()
    # ld.add_action(marker_1_node)

    marker_names = [f'marker_{m+1}' for m in range(20)]

    actions = []

    for marker in marker_names:
        actions.append(
            Node(
                package='warehouse_tasker',
                executable='marker_mapper_node',
                output='screen',
                parameters=[{'marker_name': marker}]
            )
        )

    ld = LaunchDescription(actions)

    return ld
