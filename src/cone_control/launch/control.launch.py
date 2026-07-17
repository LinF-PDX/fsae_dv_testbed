import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cone_control'),
        'config', 'control.yaml')

    return LaunchDescription([
        Node(
            package='cone_control',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='cone_control',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[params],
        ),
    ])
