import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cone_planner'),
        'config', 'planner.yaml')

    return LaunchDescription([
        Node(
            package='cone_planner',
            executable='delaunay_planner',
            name='delaunay_planner',
            output='screen',
            parameters=[params],
        ),
    ])
