"""Full autonomy bringup: LiDAR -> perception -> planning -> control -> CAN.

Starts the complete stack:
  - rslidar_sdk            (LiDAR driver -> /rslidar_points)
  - cone_detector          (+ static TF, cone_mapper) -> /cones/observed
  - cone_planner           (Delaunay centerline)      -> /path
  - cone_control           (pure_pursuit -> /cmd/auto, cmd_mux -> /cmd)
  - robot_teleop           (gamepad -> /cmd/manual, manual override)
  - can_bridge             (/cmd -> CAN -> STM32)

Press SPACE (in this terminal) to engage autonomy and hand control to
pure_pursuit. Press SPACE again to drop back to manual teleop. See the
autonomy keyboard latch section below for why this lives in this file
instead of as its own node.

Prerequisite (run once per boot):
    sudo ip link set can0 up type can bitrate 500000

Usage:
    ros2 launch robot_bringup autonomy_bringup.launch.py
    ros2 launch robot_bringup autonomy_bringup.launch.py lidar:=false   # e.g. replaying a bag
"""

import os
import sys
import termios
import threading
import tty

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


# --- Autonomy keyboard latch --------------------------------------------
#
# cmd_mux picks /cmd/auto vs /cmd/manual based on the latched /autonomy_enable
# topic below, toggled by SPACE: press once to engage autonomy, press again
# to return to manual. It has to run as a background thread *inside this
# launch process* rather than as its own Node: nodes started by `ros2 launch`
# are spawned as subprocesses wired to a pipe, not this terminal's tty, so
# they never see the keypress. This process is the one the user's shell
# actually invoked, so it still owns the real terminal.
_AUTONOMY_ENABLE_TOPIC = '/autonomy_enable'
_LATCH_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _run_autonomy_keyboard_latch():
    if not sys.stdin.isatty():
        print('autonomy keyboard latch: stdin is not a terminal -- SPACE toggle '
              'disabled, autonomy_enable stays false (manual only)')
        return

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('autonomy_keyboard_latch')
    pub = node.create_publisher(Bool, _AUTONOMY_ENABLE_TOPIC, _LATCH_QOS)

    enabled = False
    pub.publish(Bool(data=enabled))  # latch the initial "manual" state

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # single keypresses, no Enter needed; Ctrl+C still works
        print('\n>>> Press SPACE to engage autonomy. Press SPACE again for manual. <<<\n')
        while True:
            if sys.stdin.read(1) == ' ':
                enabled = not enabled
                pub.publish(Bool(data=enabled))
                print('>>> AUTONOMY ENGAGED -- press SPACE to return to manual' if enabled
                      else '>>> MANUAL CONTROL -- press SPACE to engage autonomy')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def _start_autonomy_keyboard_latch(context, *args, **kwargs):
    threading.Thread(target=_run_autonomy_keyboard_latch, daemon=True).start()
    return []


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

        # --- Autonomy engage/disengage latch: SPACE toggles it (see above) ---
        OpaqueFunction(function=_start_autonomy_keyboard_latch),
    ])
