"""Teleop bringup: gamepad -> CAN -> STM32.

Starts everything needed to drive the robot by hand:
  - can_bridge (+ ros2_socketcan sender/receiver, auto-activated)
  - robot_teleop (joy_node + joy_to_ackermann -> /cmd/manual)
  - cmd_mux (/cmd/manual -> /cmd when autonomy button not held)

No LiDAR, perception, or planning -- those are only needed for autonomy.

Prerequisite (run once per boot, needs sudo so it is NOT done here):
    sudo ip link set can0 up type can bitrate 500000

Usage:
    ros2 launch robot_bringup teleop_bringup.launch.py
    ros2 launch robot_bringup teleop_bringup.launch.py interface:=can1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interface = LaunchConfiguration('interface')

    can_bridge_launch = os.path.join(
        get_package_share_directory('can_bridge'), 'launch', 'can_bridge.launch.py')
    teleop_launch = os.path.join(
        get_package_share_directory('robot_teleop'), 'launch', 'teleop.launch.py')
    control_params = os.path.join(
        get_package_share_directory('cone_control'), 'config', 'control.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface', default_value='can0',
            description='SocketCAN interface name'),

        # CAN: /cmd <-> STM32
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(can_bridge_launch),
            launch_arguments={'interface': interface}.items(),
        ),

        # Gamepad -> /cmd/manual
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
        ),

        # Mux: /cmd/manual -> /cmd  (no /cmd/auto publisher in this mode)
        Node(
            package='cone_control',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[control_params],
        ),
    ])
