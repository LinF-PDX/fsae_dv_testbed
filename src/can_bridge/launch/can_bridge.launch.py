import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('can_bridge'),
        'config', 'can_bridge.yaml')

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='SocketCAN interface name')

    return LaunchDescription([
        interface_arg,

        # ros2_socketcan: CAN -> ROS (publishes can_msgs/Frame)
        Node(
            package='ros2_socketcan',
            executable='socket_can_receiver_node_exe',
            name='can_receiver',
            output='screen',
            parameters=[{'interface': LaunchConfiguration('interface')}],
            remappings=[('~/from_can_bus', '/from_can_bus')],
        ),

        # ros2_socketcan: ROS -> CAN (subscribes can_msgs/Frame)
        Node(
            package='ros2_socketcan',
            executable='socket_can_sender_node_exe',
            name='can_sender',
            output='screen',
            parameters=[{'interface': LaunchConfiguration('interface')}],
            remappings=[('~/to_can_bus', '/to_can_bus')],
        ),

        # Our bridge: /cmd <-> CAN frames
        Node(
            package='can_bridge',
            executable='can_bridge_node',
            name='can_bridge',
            output='screen',
            parameters=[params],
        ),
    ])
