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
  3. Slew-rate limit steering and speed against the previously published
     values, using the actual measured tick period (not an assumed one).
  4. Publish the result, plus an RViz MarkerArray for the lookahead point
     and applied steering.

Safety: publishes on a fixed timer (feeds the firmware watchdog). The planner
publishes a path every scan, including empty ones on frames with too few
cones to triangulate -- treating those as "path gone" caused the command to
flicker between the cruise speed and zero every time a single frame dropped
out. Instead, on_path only caches NON-EMPTY paths; on_tick's freshness check
against that cache is what actually decides "stop": if no non-empty path has
arrived for path_timeout seconds, the target speed/steer drop to zero (and
slew-limit down from there, same as any other command change). Autonomy is
just another publisher to the command chain; empty/stale path == stop.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


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
        self.max_steering_rate = self.declare_parameter('max_steering_rate', 1.0).value
        self.max_speed_rate = self.declare_parameter('max_speed_rate', 0.5).value
        self.publish_markers = self.declare_parameter('publish_markers', True).value
        out_topic = self.declare_parameter('output_topic', '/cmd/auto').value

        self.path_pts = None
        self.last_path_time = self.get_clock().now()
        self.path_frame_id = 'base_link'

        # State carried between ticks for slew-rate limiting and edge-triggered
        # logging (see on_tick / _slew_limit).
        self.last_steer = 0.0
        self.last_speed = 0.0
        self.last_tick_time = self.get_clock().now()
        self._path_was_ok = False

        self.sub = self.create_subscription(Path, '/path', self.on_path, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pure_pursuit/markers', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.on_tick)

        self.get_logger().info(
            f'pure_pursuit up: lookahead={self.lookahead:.2f} m, '
            f'wheelbase={self.wheelbase:.2f} m, speed={self.speed:.2f} m/s, '
            f'max_steering_rate={self.max_steering_rate:.2f} rad/s, '
            f'max_speed_rate={self.max_speed_rate:.2f} m/s^2, '
            f'publishing {out_topic}')

    def on_path(self, msg: Path):
        # Ignore empty paths entirely -- keep driving on the last cached
        # non-empty path. on_tick's freshness check against last_path_time
        # is what decides when to actually stop.
        if len(msg.poses) == 0:
            return
        self.path_pts = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.last_path_time = self.get_clock().now()
        if msg.header.frame_id:
            self.path_frame_id = msg.header.frame_id

    def on_tick(self):
        now = self.get_clock().now()

        dt = (now - self.last_tick_time).nanoseconds / 1e9
        if dt <= 0.0:
            # Clock hiccup (e.g. sim time jump/reset) -- fall back to the
            # nominal tick period rather than dividing by, or limiting to, zero.
            dt = 1.0 / self.control_rate
        self.last_tick_time = now

        target_speed, target_steer = 0.0, 0.0
        lookahead_pt = None

        fresh = (now - self.last_path_time).nanoseconds < self.path_timeout * 1e9
        path_ok = fresh and self.path_pts is not None and len(self.path_pts) >= self.min_points

        if path_ok:
            target = self.find_lookahead(self.path_pts)
            if target is not None:
                x, y = float(target[0]), float(target[1])
                ld = math.hypot(x, y)
                if ld > 1e-3:
                    alpha = math.atan2(y, x)
                    target_steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
                    target_steer = max(-self.max_steer, min(self.max_steer, target_steer))
                    target_speed = self.speed
                    lookahead_pt = (x, y)

        if path_ok and not self._path_was_ok:
            self.get_logger().info(
                'pure_pursuit: valid path resumed, driving', throttle_duration_sec=1.0)
        elif not path_ok and self._path_was_ok:
            self.get_logger().warn(
                'pure_pursuit: path lost, stopping', throttle_duration_sec=1.0)
        self._path_was_ok = path_ok

        steer = self._slew_limit(target_steer, self.last_steer, self.max_steering_rate * dt)
        speed = self._slew_limit(target_speed, self.last_speed, self.max_speed_rate * dt)
        self.last_steer = steer
        self.last_speed = speed

        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steer)
        self.pub.publish(cmd)

        if self.publish_markers:
            self.publish_marker_array(lookahead_pt, steer, speed)

    @staticmethod
    def _slew_limit(target, prev, max_delta):
        # Linear rate limiter: moves prev toward target by at most max_delta.
        # When |target - prev| <= max_delta it snaps straight to target, so
        # the output actually reaches (not just approaches) zero.
        max_delta = abs(max_delta)
        diff = target - prev
        if diff > max_delta:
            return prev + max_delta
        if diff < -max_delta:
            return prev - max_delta
        return target

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

    # ------------------------------------------------------------------
    def publish_marker_array(self, lookahead_pt, steer, speed):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        zero_lifetime = Duration(sec=0, nanosec=0)

        clear = Marker()
        clear.header.frame_id = self.path_frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        if lookahead_pt is not None:
            x, y = lookahead_pt

            sphere = Marker()
            sphere.header.frame_id = self.path_frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'pure_pursuit'
            sphere.id = 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.55, 0.0, 0.9
            sphere.lifetime = zero_lifetime
            arr.markers.append(sphere)

            line = Marker()
            line.header.frame_id = self.path_frame_id
            line.header.stamp = stamp
            line.ns = 'pure_pursuit'
            line.id = 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.9, 0.0, 0.8
            line.pose.orientation.w = 1.0
            line.points = [Point(x=0.0, y=0.0, z=0.02), Point(x=x, y=y, z=0.02)]
            line.lifetime = zero_lifetime
            arr.markers.append(line)

            arrow_len = 0.4
            arrow = Marker()
            arrow.header.frame_id = self.path_frame_id
            arrow.header.stamp = stamp
            arrow.ns = 'pure_pursuit'
            arrow.id = 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.02  # shaft diameter
            arrow.scale.y = 0.04  # head diameter
            arrow.scale.z = 0.06  # head length
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.0, 0.0, 0.9
            arrow.pose.orientation.w = 1.0
            arrow.points = [
                Point(x=0.0, y=0.0, z=0.02),
                Point(x=arrow_len * math.cos(steer), y=arrow_len * math.sin(steer), z=0.02),
            ]
            arrow.lifetime = zero_lifetime
            arr.markers.append(arrow)

            arr.markers.append(self._text_marker(
                stamp,
                f'steer: {math.degrees(steer):+.1f} deg\nspeed: {speed:.2f} m/s',
                (1.0, 1.0, 1.0)))
        else:
            arr.markers.append(self._text_marker(stamp, 'NO PATH', (1.0, 0.2, 0.2)))

        self.marker_pub.publish(arr)

    def _text_marker(self, stamp, text, color):
        m = Marker()
        m.header.frame_id = self.path_frame_id
        m.header.stamp = stamp
        m.ns = 'pure_pursuit'
        m.id = 4
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 1.0
        m.text = text
        m.lifetime = Duration(sec=0, nanosec=0)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
