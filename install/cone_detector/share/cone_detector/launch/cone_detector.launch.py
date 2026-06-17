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
        # 45-degree front mount: dome faces up-forward, scan-pattern pole at
        # 45 deg elevation (its sparse central hole points at the ceiling, not
        # at far cones). Mounted at the front of the robot, ~10 cm above floor.
        # MEASURE and update: --x (fore-aft offset of sensor optical centre
        # from base_link origin) and --z (height of optical centre above floor).
        # If the floor ends up tilted or above the robot in RViz, flip the
        # pitch sign to -0.7854 and re-check.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0.15', '--y', '0', '--z', '0.10',
                '--yaw', '0', '--pitch', '0.7854', '--roll', '0',
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
        Node(
            package='cone_detector',
            executable='cone_mapper_node',
            name='cone_mapper',
            output='screen',
            parameters=[params],
        ),
    ])