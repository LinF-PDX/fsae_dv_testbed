#!/usr/bin/env python3
"""Pure-pursuit path tracker.

Subscribes to a local path (/path, nav_msgs/Path, in base_link) and publishes
drive commands (/cmd/auto, AckermannDriveStamped). The path comes from the
Delaunay planner and is already in the robot frame, so the robot is at the
origin facing +x -- the geometry is trivial.

Method each control tick:
  1. Find the lookahead point: the first path point at least lookahead_distance
     from the robot; if the path is shorter, use its last point.
  2. Pure-pursuit steering: with the target at (x, y) in base_link,
        alpha = atan2(y, x)                 (bearing to target)
        delta = atan2(2 * L * sin(alpha), ld)   (bicycle steering angle)
     where L = wheelbase and ld = distance to the lookahead point.
  3. Publish speed (constant, configurable) + clamped steering.

Safety: publishes on a fixed timer (feeds the firmware watchdog) and emits a
ZERO command whenever the path is empty, too short, or stale. Autonomy is
just another publisher to the command chain; empty path == stop.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        self.lookahead = self.declare_parameter('lookahead_distance', 0.6).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.25).value  # MEASURE
        self.max_steer = self.declare_parameter('max_steering_angle', 0.5).value
        self.speed = self.declare_parameter('speed', 0.3).value           # start slow
        self.control_rate = self.declare_parameter('control_rate', 50.0).value
        self.path_timeout = self.declare_parameter('path_timeout', 0.5).value
        self.min_points = self.declare_parameter('min_path_points', 2).value
        out_topic = self.declare_parameter('output_topic', '/cmd/auto').value

        self.path_pts = None
        self.last_path_time = self.get_clock().now()

        self.sub = self.create_subscription(Path, '/path', self.on_path, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.on_tick)

        self.get_logger().info(
            f'pure_pursuit up: lookahead={self.lookahead:.2f} m, '
            f'wheelbase={self.wheelbase:.2f} m, speed={self.speed:.2f} m/s, '
            f'publishing {out_topic}')

    def on_path(self, msg: Path):
        self.path_pts = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.last_path_time = self.get_clock().now()

    def on_tick(self):
        speed, steer = 0.0, 0.0  # safe default

        fresh = (self.get_clock().now() - self.last_path_time).nanoseconds \
            < self.path_timeout * 1e9

        if fresh and self.path_pts is not None and len(self.path_pts) >= self.min_points:
            target = self.find_lookahead(self.path_pts)
            if target is not None:
                x, y = float(target[0]), float(target[1])
                ld = math.hypot(x, y)
                if ld > 1e-3:
                    alpha = math.atan2(y, x)
                    steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
                    steer = max(-self.max_steer, min(self.max_steer, steer))
                    speed = self.speed

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steer)
        self.pub.publish(cmd)

    def find_lookahead(self, pts):
        # First point at/beyond the lookahead radius; else the farthest point.
        dists = np.linalg.norm(pts, axis=1)
        ahead = pts[pts[:, 0] > 0.0]  # only points in front
        if len(ahead) == 0:
            return None
        d_ahead = np.linalg.norm(ahead, axis=1)
        beyond = ahead[d_ahead >= self.lookahead]
        if len(beyond) > 0:
            # closest of those beyond the radius = smoothest choice
            return beyond[np.argmin(np.linalg.norm(beyond, axis=1))]
        # path shorter than lookahead: aim at the farthest available point
        return ahead[np.argmax(d_ahead)]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
