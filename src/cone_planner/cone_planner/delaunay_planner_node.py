#!/usr/bin/env python3
"""Delaunay-triangulation local planner.

Consumes RAW per-frame cone detections (/cones/observed, PoseArray, in the
robot frame) and publishes a local centerline path (/path, nav_msgs/Path)
for a pure-pursuit controller.

This is the FSD "exploration / reactive" planner: it needs no map and no
odometry, because everything is done fresh per scan in the vehicle frame.
Per-frame detection flicker is tolerated -- the downstream controller only
chases one lookahead point, and a missing frame simply repeats or empties
the path (controller must treat an empty path as STOP).

Method per scan:
  1. Delaunay-triangulate the 2D cone positions.
  2. Collect unique triangle edges.
  3. Keep "crossing" edges by LENGTH: edges between opposite track
     boundaries have length ~ track width; edges along one boundary have
     length ~ cone spacing. Requires the track to be laid out so these two
     bands do not overlap (e.g. spacing 0.4 m, width 0.9-1.2 m).
  4. Midpoints of kept edges are centerline waypoints.
  5. Order waypoints by a greedy nearest-neighbour walk starting at the
     robot, stopping at any large gap (disconnected remnants are dropped).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial import Delaunay, QhullError


class DelaunayPlanner(Node):

    def __init__(self):
        super().__init__('delaunay_planner')

        # Crossing-edge length band. MUST match your track design:
        # boundary cone spacing < min_edge_len < track width < max_edge_len.
        self.min_edge_len = self.declare_parameter('min_edge_len', 0.6).value
        self.max_edge_len = self.declare_parameter('max_edge_len', 1.5).value

        # Waypoint chain: stop if the next nearest midpoint is farther than
        # this (prevents jumping across the track to a disconnected remnant).
        self.max_gap = self.declare_parameter('max_waypoint_gap', 0.5).value

        # Only plan through midpoints ahead of the robot (x > this).
        self.min_forward_x = self.declare_parameter('min_forward_x', 0.0).value

        # Cap the local path length (waypoints). Local planner needs only
        # a few metres of path; the rest is noise at range.
        self.max_waypoints = self.declare_parameter('max_waypoints', 5).value

        self.publish_markers = self.declare_parameter('publish_markers', True).value

        # Separate from publish_markers: the triangulation mesh draws every
        # Delaunay edge (not just the few kept as centerline), so it's the
        # more expensive of the two to build/publish per frame. Independent
        # toggle so it can be switched off for real runs while keeping the
        # (cheaper) centerline markers on.
        self.publish_triangulation = self.declare_parameter(
            'publish_triangulation', True).value

        self.sub = self.create_subscription(
            PoseArray, '/cones/observed', self.on_cones, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        self.triangulation_pub = self.create_publisher(
            MarkerArray, '/triangulation_markers', 10)

        self.get_logger().info(
            f'delaunay_planner up: crossing edges {self.min_edge_len:.2f}-'
            f'{self.max_edge_len:.2f} m, max gap {self.max_gap:.2f} m')

    # ------------------------------------------------------------------
    def on_cones(self, msg: PoseArray):
        pts = np.array([[p.position.x, p.position.y] for p in msg.poses])

        waypoints, all_edges, crossing_edges = [], [], []
        if len(pts) >= 4:
            waypoints, all_edges, crossing_edges = self.plan(pts)

        self.publish_path(msg, waypoints)
        if self.publish_markers:
            self.publish_path_markers(msg, waypoints)
        if self.publish_triangulation:
            self.publish_triangulation_markers(msg, pts, all_edges, crossing_edges)

    # ------------------------------------------------------------------
    def plan(self, pts: np.ndarray):
        # 1-2. Triangulate and collect unique edges.
        try:
            tri = Delaunay(pts)
        except QhullError:
            # Degenerate input (collinear cones etc.) -- no plan this frame.
            return [], [], []

        edges = set()
        for simplex in tri.simplices:
            for i in range(3):
                a, b = int(simplex[i]), int(simplex[(i + 1) % 3])
                edges.add((min(a, b), max(a, b)))

        # 3-4. Length-filter to crossing edges; take midpoints. Keep the
        # (a, b) index pairs alongside each midpoint so the crossing edges
        # can be drawn later (see publish_triangulation_markers) -- not just
        # the point they collapse to.
        mids = []
        crossing_edges = []
        for a, b in edges:
            d = float(np.linalg.norm(pts[a] - pts[b]))
            if self.min_edge_len <= d <= self.max_edge_len:
                m = (pts[a] + pts[b]) / 2.0
                if m[0] > self.min_forward_x:
                    mids.append(m)
                    crossing_edges.append((a, b))

        if not mids:
            return [], list(edges), crossing_edges

        # 5. Greedy nearest-neighbour ordering from the robot (origin).
        ordered = []
        cur = np.zeros(2)
        remaining = list(mids)
        while remaining and len(ordered) < self.max_waypoints:
            dists = [float(np.linalg.norm(m - cur)) for m in remaining]
            i = int(np.argmin(dists))
            if ordered and dists[i] > self.max_gap:
                break  # rest is disconnected -- drop it
            cur = remaining.pop(i)
            ordered.append(cur)

        return ordered, list(edges), crossing_edges

    # ------------------------------------------------------------------
    def publish_path(self, src_msg: PoseArray, waypoints):
        path = Path()
        path.header = src_msg.header  # same frame + stamp as the detections
        for w in waypoints:
            ps = PoseStamped()
            ps.header = src_msg.header
            ps.pose.position.x = float(w[0])
            ps.pose.position.y = float(w[1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    # ------------------------------------------------------------------
    def publish_path_markers(self, src_msg: PoseArray, waypoints):
        arr = MarkerArray()

        clear = Marker()
        clear.header = src_msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        if waypoints:
            line = Marker()
            line.header = src_msg.header
            line.ns = 'centerline'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.03
            line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.5, 1.0, 0.9
            line.pose.orientation.w = 1.0
            for w in waypoints:
                p = Point()
                p.x, p.y, p.z = float(w[0]), float(w[1]), 0.05
                line.points.append(p)
            arr.markers.append(line)

            for i, w in enumerate(waypoints):
                s = Marker()
                s.header = src_msg.header
                s.ns = 'waypoints'
                s.id = i + 1
                s.type = Marker.SPHERE
                s.action = Marker.ADD
                s.pose.position.x = float(w[0])
                s.pose.position.y = float(w[1])
                s.pose.position.z = 0.05
                s.pose.orientation.w = 1.0
                s.scale.x = s.scale.y = s.scale.z = 0.08
                s.color.r, s.color.g, s.color.b, s.color.a = 0.1, 0.5, 1.0, 0.9
                arr.markers.append(s)

        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------
    def publish_triangulation_markers(self, src_msg, pts, all_edges, crossing_edges):
        """Draw the raw Delaunay mesh over the cones, with the crossing
        edges (the ones that became centerline waypoints) highlighted --
        this is purely for visualization, to make the algorithm's geometry
        visible on top of /path_markers rather than a black box."""
        arr = MarkerArray()

        clear = Marker()
        clear.header = src_msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        if all_edges:
            mesh = Marker()
            mesh.header = src_msg.header
            mesh.ns = 'triangulation_mesh'
            mesh.id = 0
            mesh.type = Marker.LINE_LIST
            mesh.action = Marker.ADD
            mesh.scale.x = 0.015
            mesh.color.r, mesh.color.g, mesh.color.b, mesh.color.a = 0.8, 0.8, 0.8, 0.5
            mesh.pose.orientation.w = 1.0
            for a, b in all_edges:
                mesh.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=0.02))
                mesh.points.append(Point(x=float(pts[b][0]), y=float(pts[b][1]), z=0.02))
            arr.markers.append(mesh)

        if crossing_edges:
            kept = Marker()
            kept.header = src_msg.header
            kept.ns = 'triangulation_crossing'
            kept.id = 1
            kept.type = Marker.LINE_LIST
            kept.action = Marker.ADD
            kept.scale.x = 0.025
            kept.color.r, kept.color.g, kept.color.b, kept.color.a = 1.0, 0.9, 0.0, 0.9
            kept.pose.orientation.w = 1.0
            for a, b in crossing_edges:
                kept.points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=0.04))
                kept.points.append(Point(x=float(pts[b][0]), y=float(pts[b][1]), z=0.04))
            arr.markers.append(kept)

        self.triangulation_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DelaunayPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
