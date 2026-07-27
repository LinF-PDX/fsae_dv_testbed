"""Full autonomy bringup: LiDAR -> perception -> planning -> control -> CAN.

Starts the complete stack:
  - rslidar_sdk            (LiDAR driver -> /rslidar_points)
  - cone_detector          (+ static TF, cone_mapper) -> /cones/observed
  - cone_planner           (Delaunay centerline)      -> /path
  - cone_control           (pure_pursuit -> /cmd/auto, cmd_mux -> /cmd)
  - robot_teleop           (gamepad -> /cmd/manual, manual override)
  - can_bridge             (/cmd -> CAN -> STM32)

HOLD the autonomy button (A) on the gamepad to hand control to pure_pursuit.
RELEASE it to fall back to manual teleop. Release everything to stop.

Prerequisite (run once per boot):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up

Usage:
    ros2 launch robot_bringup autonomy_bringup.launch.py
    ros2 launch robot_bringup autonomy_bringup.launch.py lidar:=false   # e.g. replaying a bag
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    use_lidar = LaunchConfiguration('lidar')

    lidar_launch = os.path.join(
        get_package_share_directory('rslidar_sdk'), 'launch', 'start.py')
    detector_launch = os.path.join(
        get_package_share_directory('cone_detector'), 'launch', 'cone_detector.launch.py')
    planner_launch = os.path.join(
        get_package_share_directory('cone_planner'), 'launch', 'planner.launch.py')
    control_launch = os.path.join(
        get_package_share_directory('cone_control'), 'launch', 'control.launch.py')
    teleop_launch = os.path.join(
        get_package_share_directory('robot_teleop'), 'launch', 'teleop.launch.py')
    can_bridge_launch = os.path.join(
        get_package_share_directory('can_bridge'), 'launch', 'can_bridge.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface', default_value='can0',
            description='SocketCAN interface name'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='Start the LiDAR driver (set false when replaying a bag)'),

        # --- Sensing ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(use_lidar),
        ),

        # --- Perception (includes static TF + cone_mapper) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detector_launch),
        ),

        # --- Planning ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
        ),

        # --- Control: pure_pursuit -> /cmd/auto, cmd_mux -> /cmd ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
        ),

        # --- Manual override / deadman ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
        ),

        # --- Actuation ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(can_bridge_launch),
            launch_arguments={'interface': interface}.items(),
        ),
    ])
