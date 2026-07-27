# cone_planner

Delaunay-triangulation local centerline planner. Consumes raw per-frame cone
detections and publishes a local path for `pure_pursuit` to track — no map,
no odometry, everything computed fresh per scan in the vehicle frame.

```
/cones/observed (PoseArray, base_link)
   -> delaunay_planner_node
        -> /path (nav_msgs/Path)
        `-> /path_markers (visualization_msgs/MarkerArray)
```

This is the FSD "exploration / reactive" planning style. Per-frame detection
flicker is tolerated: the downstream controller only chases one lookahead
point, and a missing frame simply repeats or empties the path (`pure_pursuit`
treats an empty/stale path as a reason to stop — see
[`cone_control`'s README](../cone_control/README.md)).

## Algorithm, per scan

1. Delaunay-triangulate the 2D cone positions.
2. Collect unique triangle edges.
3. Keep **crossing edges** by length: edges between opposite track boundaries
   have length ≈ track width; edges along one boundary have length ≈ cone
   spacing. This only works if the track is laid out so those two length
   bands don't overlap — **you must design the track to satisfy**:

   ```
   boundary cone spacing  <  min_edge_len  <  track width  <  max_edge_len
   ```

   e.g. spacing 0.4 m, width 0.9–1.2 m -> band `[0.6, 1.5]` separates cleanly.
4. Midpoints of the kept edges become centerline waypoints.
5. Order waypoints with a greedy nearest-neighbour walk starting at the
   robot, stopping at any large gap (disconnected remnants are dropped, not
   jumped across).

Needs at least 4 cones in view to triangulate at all; fewer than that (or a
degenerate/collinear frame) publishes an empty path for that scan.

## Configuration (`config/planner.yaml`)

| Key | Default | Description |
|---|---|---|
| `min_edge_len` / `max_edge_len` | `0.8` / `1.5 m` | Crossing-edge length band — must satisfy the track-design constraint above |
| `max_waypoint_gap` | `0.4 m` | Stop chaining waypoints past this gap |
| `min_forward_x` | `0.0 m` | Ignore midpoints behind/at the robot |
| `max_waypoints` | `8` | Local path length cap — a local planner only needs a few metres |
| `publish_markers` | `true` | Publish `/path_markers` |

## Run standalone

Needs `/cones/observed` already publishing (from `cone_detector`, or a
replayed bag):

```bash
ros2 launch cone_planner planner.launch.py
```

In RViz, add `/path_markers` (blue centerline + waypoint spheres) alongside
`cone_detector`'s `/cones/markers` to check the plan against the actual
detections.

## Tuning

If the path is empty or wrong shape with cones clearly visible in
`/cones/markers`:

- **No path at all**: fewer than 4 cones in the frame, or `min_edge_len`/
  `max_edge_len` don't bracket any real edges — check actual cone spacing and
  track width against the configured band.
- **Path jumps across the track**: `max_edge_len` too loose, admitting
  boundary-to-boundary diagonals as "crossing" edges — tighten it.
- **Path stops short / breaks up**: `max_waypoint_gap` too tight for your
  actual cone spacing, or too few cones detected at range (check
  `cone_detector`'s `min/max_cluster_size` — see its
  [README](../cone_detector/README.md)).
