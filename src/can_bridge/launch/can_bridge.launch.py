import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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

    # --- Our bridge node (normal node, not lifecycle) ---
    bridge = Node(
        package='can_bridge',
        executable='can_bridge_node',
        name='can_bridge',
        output='screen',
        parameters=[params],
    )

    # Drive each lifecycle node unconfigured -> inactive -> active as soon as
    # it is actually ready, instead of guessing a fixed wall-clock delay.
    #
    # The previous approach fired `ros2 lifecycle set ... configure/activate`
    # from TimerActions at t=2s/t=3s. That raced with node startup: under the
    # full autonomy stack (lidar driver + perception + planning all starting
    # at once), can_receiver/can_sender routinely weren't discoverable yet at
    # t=2s, the CLI calls failed outright ("Node not found" / "Unknown
    # transition requested"), and the nodes were stuck "unconfigured"
    # forever -- so /to_can_bus was silently dropped and the robot never
    # moved, even though the rest of the graph (and RViz) looked fine.
    def bring_up(node):
        return [
            RegisterEventHandler(OnProcessStart(
                target_action=node,
                on_start=EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            )),
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=node,
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))],
            )),
        ]

    return LaunchDescription([
        interface_arg,
        can_receiver,
        can_sender,
        bridge,
        *bring_up(can_receiver),
        *bring_up(can_sender),
    ])
