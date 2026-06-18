import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('can_bridge'),
        'config', 'can_bridge.yaml')

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='SocketCAN interface name')

    # --- ros2_socketcan: CAN receiver (lifecycle node) ---
    can_receiver = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='can_receiver',
        namespace='',
        output='screen',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'interval_sec': 0.1,   # 100 ms; STM32 sends at ~50 Hz (20 ms)
        }],
        remappings=[('~/from_can_bus', '/from_can_bus')],
    )

    # --- ros2_socketcan: CAN sender (lifecycle node) ---
    can_sender = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='can_sender',
        namespace='',
        output='screen',
        parameters=[{'interface': LaunchConfiguration('interface')}],
        remappings=[('~/to_can_bus', '/to_can_bus')],
    )

    # Auto-configure + activate after a short delay so the nodes are up.
    # This is the equivalent of the four manual `ros2 lifecycle set` commands.
    activate_nodes = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set', '/can_receiver', 'configure']),
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set', '/can_sender', 'configure']),
        ],
    )
    activate_nodes_2 = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set', '/can_receiver', 'activate']),
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set', '/can_sender', 'activate']),
        ],
    )

    # --- Our bridge node (normal node, not lifecycle) ---
    bridge = Node(
        package='can_bridge',
        executable='can_bridge_node',
        name='can_bridge',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        interface_arg,
        can_receiver,
        can_sender,
        bridge,
        activate_nodes,     # configure both at t=2s
        activate_nodes_2,   # activate both at t=3s
    ])