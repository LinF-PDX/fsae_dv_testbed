# fsae_dv_testbed

A ROS 2 workspace for FSAE Driverless Vehicle development. The stack acquires
point clouds from a RoboSense Airy LiDAR, detects and maps traffic cones
(inverted cups) with a classical PCL pipeline, plans a local centerline
through them, and drives the vehicle with a pure-pursuit controller — with a
gamepad always available to drive manually instead. `robot_bringup` starts
the whole thing with a single launch command; see
**[Run the robot](#run-the-robot)** below.

This file is the outline. Each package's own README has the pipeline
details, node parameters, tuning guides, and standalone run/test commands —
follow the links in the table below.

## Packages

| Package | Description | Docs |
|---|---|---|
| `cone_detector` | LiDAR-based cone detection (`cone_detector_node`) and persistent cone mapping (`cone_mapper_node`) | [README](src/cone_detector/README.md) |
| `cone_planner` | Delaunay-triangulation local centerline planner (`delaunay_planner_node`) | [README](src/cone_planner/README.md) |
| `cone_control` | Pure-pursuit path tracker (`pure_pursuit`) and manual/autonomy command mux (`cmd_mux`) | [README](src/cone_control/README.md) |
| `robot_teleop` | Xbox-style gamepad teleop — `/joy` to Ackermann drive commands | [README](src/robot_teleop/README.md) |
| `can_bridge` | Bridges Ackermann drive commands to CAN frames for the STM32, and publishes encoder feedback | [README](src/can_bridge/README.md) |
| `robot_bringup` | Top-level launch files — brings up the full autonomy stack or teleop-only with one command | [README](src/robot_bringup/README.md) |
| `rslidar_sdk` | RoboSense LiDAR ROS 2 driver (submodule, supports RS-AIRY and 15+ models) | [upstream README](src/rslidar_sdk/README.md) |
| `rslidar_msg` | ROS 2 message definitions required by `rslidar_sdk` | [upstream README](src/rslidar_msg-master/README.md) |

## System Overview

```
RoboSense Airy LiDAR                                   Xbox gamepad
        |                                                    |
   rslidar_sdk_node                                      joy_node
        | /rslidar_points (PointCloud2)                      | /joy
   cone_detector_node                                        |
        |-- /cones/observed (PoseArray, base_link)            |
        |-- /cones/markers, /debug/*                          |
        |-- v (mapping/visualization only, not in the         |
        |     control loop)                                   |
        |   cone_mapper_node                                  |
        |     |-- /cones/map, /cones/map_markers               |
        v                                                     v
   delaunay_planner_node                             joy_to_ackermann_node
        |-- /path (nav_msgs/Path)                              | /cmd/manual
        |-- /path_markers                                      |
        `-- /triangulation_markers                             |
        v                                                      |
   pure_pursuit_node                                           |
        `-- /cmd/auto (AckermannDriveStamped)                  |
                        \                                     /
                         v                                   v
                              cmd_mux_node
                (SPACE key, latched: engage  -> forward /cmd/auto
                             disengage -> forward /cmd/manual)
                                    |
                              /cmd (AckermannDriveStamped)
                                    |
                            can_bridge_node
                                    |-- /to_can_bus -> STM32
                                    `-- /from_can_bus -> /encoder
```

`cone_mapper_node`'s confirmed cone map (`/cones/map`) is not currently
consumed by the planner — `delaunay_planner_node` plans directly off the raw
per-scan `/cones/observed`. The mapper exists today for visualization and as
a stepping stone toward map-based planning.

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

## Run the robot

`robot_bringup` is the master node for the whole vehicle — it starts every
node below with one command instead of one launch per terminal. See
[`src/robot_bringup/README.md`](src/robot_bringup/README.md) for launch
arguments and what each option starts internally.

**0. One-time per boot** — bring the CAN interface up (needs `sudo`, so it's
a separate script, not part of the launch file):

```bash
~/ros2_ws/src/robot_bringup/scripts/can_up.sh can0 500000
```

**1. Full autonomy** (LiDAR -> perception -> planning -> control -> CAN, plus
teleop as a live override):

```bash
ros2 launch robot_bringup autonomy_bringup.launch.py
```

- **Press SPACE** (in the terminal running the launch command) to engage
  autonomy and hand control to pure_pursuit. **Press SPACE again** to drop
  back to manual teleop — it's a latch, not a hold. Release the gamepad
  entirely while in manual mode and the robot stops — `cmd_mux` always falls
  back to zero if neither source is fresh.
- Pass `lidar:=false` when replaying a bag instead of running the live LiDAR.

**Or, teleop only** (no LiDAR/perception/planning — just gamepad -> CAN):

```bash
ros2 launch robot_bringup teleop_bringup.launch.py
```

Both accept `interface:=<can-iface>` (default `can0`).

### Developing perception/planning without hardware

Record a drive through cones once, then iterate against the bag instead of
the live LiDAR — see [`src/cone_detector/README.md`](src/cone_detector/README.md#run)
for the record/replay workflow and what to look at in RViz.

### Something looks wrong

RViz showing detections/path but the robot not responding to teleop or
autonomy almost always means a CAN lifecycle node never reached `active` —
see the troubleshooting notes in
[`src/can_bridge/README.md`](src/can_bridge/README.md#troubleshooting) and
[`src/robot_bringup/README.md`](src/robot_bringup/README.md#troubleshooting).

## Repository Structure

```
ros2_ws/
├── src/
│   ├── cone_detector/          # Cone detection + mapping
│   ├── cone_planner/           # Delaunay local centerline planner
│   ├── cone_control/           # Pure-pursuit controller + command mux
│   ├── robot_teleop/           # Gamepad teleop
│   ├── can_bridge/             # CAN bridge to STM32
│   ├── robot_bringup/          # Top-level launch files (the entry point)
│   ├── rslidar_sdk/            # RoboSense SDK (git submodule)
│   └── rslidar_msg-master/     # Message definitions for rslidar_sdk
├── build/
├── install/
└── log/
```

Each `src/<package>/` follows the standard ROS 2 layout: `config/*.yaml` for
parameters, `launch/*.launch.py` to run it standalone, and its own `README.md`
for details.

## Submodule

`rslidar_sdk` is tracked as a git submodule. After cloning this repo, run:

```bash
git submodule update --init --recursive
```
