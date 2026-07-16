# fsae_dv_testbed

A ROS 2 workspace for FSAE Driverless Vehicle development. The stack acquires
point clouds from a RoboSense Airy LiDAR, detects and maps traffic cones
(inverted cups) with a classical PCL pipeline, plans a local centerline
through them, and drives the vehicle — either by joystick teleop or a
CAN-connected STM32 — via Ackermann drive commands.

## Packages

| Package | Description |
|---|---|
| `cone_detector` | LiDAR-based cone detection (`cone_detector_node`) and persistent cone mapping (`cone_mapper_node`) |
| `cone_planner` | Delaunay-triangulation local centerline planner (`delaunay_planner_node`) |
| `robot_teleop` | Xbox-style gamepad teleop — `/joy` to Ackermann drive commands |
| `can_bridge` | Bridges Ackermann drive commands to CAN frames for the STM32, and publishes encoder feedback |
| `rslidar_sdk` | RoboSense LiDAR ROS 2 driver (submodule, supports RS-AIRY and 15+ models) |
| `rslidar_msg` | ROS 2 message definitions required by `rslidar_sdk` |

## System Overview

```
RoboSense Airy LiDAR                          Xbox gamepad
        |                                           |
   rslidar_sdk_node                             joy_node
        | /rslidar_points (PointCloud2)             | /joy
   cone_detector_node                       joy_to_ackermann_node
        |-- /cones/observed (PoseArray)              |
        |-- /cones/markers, /debug/*                 |
        v                                            |
   cone_mapper_node                                  |
        |-- /cones/map (PoseArray, confirmed cones)  |
        `-- /cones/map_markers                       |
        |                                            |
   delaunay_planner_node                             |
        |-- /path (nav_msgs/Path)                    |
        `-- /path_markers                            |
                                                      v
                                              /cmd (AckermannDriveStamped)
                                                      |
                                              can_bridge_node
                                                      |-- /to_can_bus -> STM32
                                                      `-- /from_can_bus -> /encoder
```

`cone_detector_node` and `delaunay_planner_node` are not yet wired together
(no controller consumes `/path` yet) — today the vehicle is driven by
`robot_teleop` publishing directly to `/cmd`, with `can_bridge` as the only
consumer.

## Prerequisites

- ROS 2 (Humble or newer)
- PCL 1.12+
- libpcap (for rslidar_sdk PCAP playback)
- yaml-cpp
- Python `numpy` and `scipy` (for `cone_planner`)
- `joy` (for `robot_teleop`)
- `ros2_socketcan` and a SocketCAN interface (for `can_bridge`)

Install system dependencies:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To build a single package:

```bash
colcon build --packages-select cone_detector
```

## Run

### Live hardware

**Terminal 1** — start the LiDAR driver:

```bash
ros2 launch rslidar_sdk start.py
```

The driver publishes point clouds on `/rslidar_points` (configured in `src/rslidar_sdk/config/config.yaml`).

**Terminal 2** — start cone detection + mapping:

```bash
ros2 launch cone_detector cone_detector.launch.py
```

This also publishes the `base_link` -> `rslidar` static transform for the
current mount (front-mounted, 45° forward tilt). The launch file remaps
`/points` -> `/rslidar_points`. Parameters are loaded from
`src/cone_detector/config/params.yaml`.

**Terminal 3** — start the local planner (optional, not yet consumed by a controller):

```bash
ros2 launch cone_planner planner.launch.py
```

**Terminal 4** — drive the vehicle. Either joystick teleop:

```bash
ros2 launch robot_teleop teleop.launch.py
```

or the CAN bridge to the STM32 (bring up whichever publishes `/cmd`, not both at once):

```bash
ros2 launch can_bridge can_bridge.launch.py interface:=can0
```

`can_bridge.launch.py` also brings up and activates the `ros2_socketcan`
sender/receiver lifecycle nodes on the given interface.

### Bag-based development (recommended for perception/planning work)

Record a drive through cones:

```bash
ros2 bag record /rslidar_points /tf /tf_static -o cones_bag
```

Replay while the detector (and planner) is running:

```bash
ros2 bag play cones_bag --loop
```

Open RViz and add:
- `/rslidar_points` — raw cloud
- `/debug/cropped` — after crop + downsample
- `/debug/clusters` — clusters that passed the shape filter
- `/cones/markers` — per-scan detected cones
- `/cones/map_markers` — confirmed, persistent cone map
- `/path_markers` — planned local centerline

## Configuration

### LiDAR driver (`src/rslidar_sdk/config/config.yaml`)

Key settings:

| Key | Default | Description |
|---|---|---|
| `msg_source` | `1` | `1` = live LiDAR, `3` = PCAP file |
| `lidar_type` | `RSAIRY` | LiDAR model |
| `msop_port` | `6699` | Data port |
| `difop_port` | `7788` | Config port |
| `ros_send_point_cloud_topic` | `/rslidar_points` | Output topic |

### Cone detector + mapper (`src/cone_detector/config/params.yaml`)

Edit and re-launch — no recompile needed.

`cone_detector` (per-scan detection):

| Key | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to transform the cloud into |
| `crop_min/max_x` | `0.2–2.5 m` | Forward region of interest |
| `crop_min/max_y` | `±1.0 m` | Lateral region of interest |
| `crop_min/max_z` | `0.08–0.40 m` | z bounds for ground removal |
| `voxel_leaf` | `0.0` (disabled) | Downsample resolution |
| `cluster_tolerance` | `0.12 m` | Max gap within a cluster |
| `min/max_cluster_size` | `2–3000` | Point count gates |
| `min_top_z` | `0.14 m` | Cluster must reach above this height to count as a cone |
| `min/max_footprint` | `0.0–0.20 m` | Accepted cluster width/depth |
| `min/max_height` | `0.02–0.30 m` | Accepted cluster height |
| `publish_debug` | `true` | Publish `/debug/*` topics |

`cone_mapper` (persistent map, built on top of `/cones/observed`):

| Key | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame the map lives in |
| `association_radius` | `0.20 m` | Distance to match a detection to an existing tracked cone |
| `min_observations` | `5` | Detections needed before a cone is confirmed and published to `/cones/map` |
| `forget_unconfirmed_sec` | `2.0 s` | Unconfirmed tracks older than this are dropped |

See [`src/cone_detector/README.md`](src/cone_detector/README.md) for the full stage-by-stage tuning guide.

### Local planner (`src/cone_planner/config/planner.yaml`)

Consumes `/cones/observed` and Delaunay-triangulates the 2D cone positions
each scan; edges whose length falls in a "crossing" band (between two track
boundaries) are kept, and their midpoints become the centerline path. The
track layout must be designed so cone spacing and track width don't overlap.

| Key | Default | Description |
|---|---|---|
| `min/max_edge_len` | `0.8–1.5 m` | Crossing-edge length band; must satisfy boundary spacing < min < track width < max |
| `max_waypoint_gap` | `0.4 m` | Stop chaining waypoints past this gap (avoids jumping to disconnected remnants) |
| `min_forward_x` | `0.0 m` | Ignore midpoints behind/at the robot |
| `max_waypoints` | `8` | Local path length cap |
| `publish_markers` | `true` | Publish `/path_markers` |

### Teleop (`src/robot_teleop/config/teleop.yaml`)

| Key | Default | Description |
|---|---|---|
| `speed_axis` / `steering_axis` | `5` / `0` | `/joy` axis indices — verify with `ros2 topic echo /joy` |
| `deadman_button` | `-1` (disabled) | Button held to enable driving; `-1` disables the deadman |
| `max_speed` | `1.0 m/s` | Speed at full stick |
| `max_steering_angle` | `0.35 rad` | Steering at full stick |
| `invert_speed` / `invert_steering` | `false` / `false` | Flip if the robot drives/steers backwards from the stick |
| `publish_rate` | `50.0 Hz` | Command rate (feeds the firmware watchdog) |
| `joy_timeout` | `0.5 s` | No `/joy` for this long -> publish zero (safe stop) |

### CAN bridge (`src/can_bridge/config/can_bridge.yaml`)

Bridges `/cmd` (AckermannDriveStamped) to CAN frames via `ros2_socketcan`
(`/to_can_bus`, `/from_can_bus`), and unpacks encoder feedback to `/encoder`.
All CAN IDs and scaling are parameters so the DBC can change without
recompiling.

| Key | Default | Description |
|---|---|---|
| `cmd_can_id` / `fb_can_id` | `0x200` / `0x201` | DV_Command / DV_Feedback CAN IDs |
| `steering_scale` | `1800/pi ≈ 572.96` | rad -> DBC units (deg × 10) |
| `max_pwm` / `speed_at_max_pwm` | `1000.0` / `0.2 m/s` | m/s -> PWM calibration point |
| `max_steering_dbc` | `300.0` (±30 deg) | Clamp applied after conversion |
| `max_speed_pwm` | `1000.0` | Clamp applied after conversion |
| `cmd_topic` | `/cmd` | Input command topic (teleop and `can_bridge` both use this — run only one source at a time until a mux is added) |

## Repository Structure

```
ros2_ws/
├── src/
│   ├── cone_detector/          # Cone detection + mapping
│   │   ├── config/params.yaml
│   │   ├── launch/cone_detector.launch.py
│   │   └── src/cone_detector_node.cpp, cone_mapper_node.cpp
│   ├── cone_planner/           # Delaunay local centerline planner
│   │   ├── config/planner.yaml
│   │   ├── launch/planner.launch.py
│   │   └── cone_planner/delaunay_planner_node.py
│   ├── robot_teleop/           # Gamepad teleop
│   │   ├── config/teleop.yaml
│   │   ├── launch/teleop.launch.py
│   │   └── src/joy_to_ackermann_node.cpp
│   ├── can_bridge/             # CAN bridge to STM32
│   │   ├── config/can_bridge.yaml
│   │   ├── launch/can_bridge.launch.py
│   │   └── src/can_bridge_node.cpp
│   ├── rslidar_sdk/            # RoboSense SDK (git submodule)
│   │   ├── config/config.yaml
│   │   └── src/rs_driver/
│   └── rslidar_msg-master/     # Message definitions for rslidar_sdk
│       └── msg/RslidarPacket.msg
├── build/
├── install/
└── log/
```

## Submodule

`rslidar_sdk` is tracked as a git submodule. After cloning this repo, run:

```bash
git submodule update --init --recursive
```
