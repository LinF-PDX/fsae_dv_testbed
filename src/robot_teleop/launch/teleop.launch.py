import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robot_teleop'),
        'config', 'teleop.yaml')

    return LaunchDescription([
        # Joystick driver: publishes /joy from the controller.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,          # first controller (/dev/input/js0)
                'deadzone': 0.05,        # ignore tiny stick noise near centre
                'autorepeat_rate': 20.0, # republish state even when still
            }],
        ),
        # Translator: /joy -> /cmd (AckermannDriveStamped).
        Node(
            package='robot_teleop',
            executable='joy_to_ackermann_node',
            name='joy_to_ackermann',
            output='screen',
            parameters=[params],
        ),
    ])
