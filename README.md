# fsae_dv_testbed

A ROS 2 workspace for FSAE Driverless Vehicle perception. The stack acquires point clouds from a RoboSense Airy LiDAR and detects traffic cones (inverted cups) in real time using a classical PCL pipeline.

## Packages

| Package | Description |
|---|---|
| `cone_detector` | LiDAR-based cone detection node — crops, clusters, and shape-filters a point cloud to produce cone centroids |
| `rslidar_sdk` | RoboSense LiDAR ROS 2 driver (submodule, supports RS-AIRY and 15+ models) |
| `rslidar_msg` | ROS 2 message definitions required by `rslidar_sdk` |

## System Overview

```
RoboSense Airy LiDAR
        |
   rslidar_sdk_node
        | /rslidar_points (PointCloud2)
   cone_detector_node
        |-- /cones/observed  (geometry_msgs/PoseArray)   <- cone centroids
        |-- /cones/markers   (visualization_msgs/MarkerArray) <- RViz cylinders
        |-- /debug/cropped   (PointCloud2)  <- after crop + downsample
        `-- /debug/clusters  (PointCloud2)  <- clusters that passed shape filter
```

## Prerequisites

- ROS 2 (Humble or newer)
- PCL 1.12+
- libpcap (for rslidar_sdk PCAP playback)
- yaml-cpp

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

**Terminal 2** — start the cone detector:

```bash
ros2 launch cone_detector cone_detector.launch.py
```

The launch file remaps `/points` → `/rslidar_points` and applies a static transform for the LiDAR mount (0.18 m height, +90° pitch). Parameters are loaded from `src/cone_detector/config/params.yaml`.

### Bag-based development (recommended)

Record a drive through cones:

```bash
ros2 bag record /rslidar_points /tf /tf_static -o cones_bag
```

Replay while the detector is running:

```bash
ros2 bag play cones_bag --loop
```

Open RViz and add:
- `/rslidar_points` — raw cloud
- `/debug/cropped` — after crop + downsample
- `/debug/clusters` — clusters that passed the shape filter
- `/cones/markers` — detected cones as orange cylinders

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

### Cone detector (`src/cone_detector/config/params.yaml`)

Edit and re-launch — no recompile needed.

| Key | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to transform the cloud into |
| `crop_min/max_x/y` | `0–6 m`, `±3 m` | Region of interest in x/y |
| `crop_min/max_z` | `0.02–0.13 m` | z bounds for ground and ceiling removal |
| `voxel_leaf` | `0.02 m` | Downsample resolution |
| `cluster_tolerance` | `0.05 m` | Max gap within a cluster |
| `min/max_cluster_size` | `3–300` | Point count gates |
| `min/max_footprint` | `0.03–0.15 m` | Accepted cluster width/depth |
| `min/max_height` | `0.03–0.15 m` | Accepted cluster height |
| `publish_debug` | `true` | Publish `/debug/*` topics |

See [`src/cone_detector/README.md`](src/cone_detector/README.md) for the full stage-by-stage tuning guide.

## Repository Structure

```
ros2_ws/
├── src/
│   ├── cone_detector/          # Cone detection package
│   │   ├── config/params.yaml
│   │   ├── launch/cone_detector.launch.py
│   │   └── src/cone_detector_node.cpp
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
