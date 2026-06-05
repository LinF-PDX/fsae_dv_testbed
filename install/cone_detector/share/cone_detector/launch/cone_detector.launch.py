import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cone_detector'),
        'config', 'params.yaml')

    return LaunchDescription([
        # LiDAR mount: base_link -> rslidar.
        # Airy is rotated +90 deg about Y (pitch = 1.5708). If the floor ends up
        # ABOVE the robot in RViz, flip the sign to -1.5708. Set z to mount height.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.18',
                '--yaw', '0', '--pitch', '1.5708', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'rslidar',
            ],
        ),
        Node(
            package='cone_detector',
            executable='cone_detector_node',
            name='cone_detector',
            output='screen',
            parameters=[params],
            remappings=[('/points', '/rslidar_points')],
        ),
    ])