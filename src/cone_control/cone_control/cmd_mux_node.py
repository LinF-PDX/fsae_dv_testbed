#!/usr/bin/env python3
"""Command multiplexer: choose manual vs. autonomous drive commands.

Subscribes:
  /cmd/manual        (AckermannDriveStamped)  -- from gamepad teleop
  /cmd/auto          (AckermannDriveStamped)  -- from pure_pursuit
  /autonomy_enable   (std_msgs/Bool, latched) -- SPACE-key toggle from the
                                                  autonomy_bringup launch file

Publishes:
  /cmd         (AckermannDriveStamped)  -- to the CAN bridge

Selection (latched, not a deadman -- see autonomy_bringup.launch.py):
  - autonomy_enable == True   -> forward /cmd/auto  (robot drives itself)
  - autonomy_enable == False  -> forward /cmd/manual (you drive / it stops)
Toggling back to False instantly returns control to the human. Since the
gamepad teleop has its own deadman, releasing everything -> zero -> stop.

Safety: publishes on a fixed timer (feeds the firmware watchdog). If the
selected source is stale (no recent message), publishes ZERO.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

# Matches the publisher QoS in robot_bringup's autonomy_bringup.launch.py so
# a cmd_mux that (re)starts after the toggle still picks up the latched state.
_LATCH_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')

        self.control_rate = self.declare_parameter('control_rate', 50.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.3).value

        self.manual = None
        self.manual_t = self.get_clock().now()
        self.auto = None
        self.auto_t = self.get_clock().now()
        self.auto_enabled = False

        self.create_subscription(
            AckermannDriveStamped, '/cmd/manual', self.on_manual, 10)
        self.create_subscription(
            AckermannDriveStamped, '/cmd/auto', self.on_auto, 10)
        self.create_subscription(
            Bool, '/autonomy_enable', self.on_autonomy_enable, _LATCH_QOS)

        self.pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.timer = self.create_timer(1.0 / self.control_rate, self.on_tick)

        self.get_logger().info(
            'cmd_mux up: press SPACE in the launch terminal for AUTONOMY, '
            'again for MANUAL. Publishing /cmd.')

    def on_manual(self, msg):
        self.manual = msg
        self.manual_t = self.get_clock().now()

    def on_auto(self, msg):
        self.auto = msg
        self.auto_t = self.get_clock().now()

    def on_autonomy_enable(self, msg):
        was = self.auto_enabled
        self.auto_enabled = msg.data
        if self.auto_enabled != was:
            self.get_logger().info(
                'AUTONOMY engaged' if self.auto_enabled else 'MANUAL control')

    def fresh(self, t):
        return (self.get_clock().now() - t).nanoseconds < self.cmd_timeout * 1e9

    def on_tick(self):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        # safe default: zero

        if self.auto_enabled and self.auto is not None and self.fresh(self.auto_t):
            out.drive = self.auto.drive
        elif self.manual is not None and self.fresh(self.manual_t):
            out.drive = self.manual.drive
        # else: leave zero

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
