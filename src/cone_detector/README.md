# cone_detector

Two nodes: `cone_detector_node` detects cones (inverted cups) in a LiDAR
point cloud and publishes their 2D positions per scan. `cone_mapper_node`
tracks those detections over time into a persistent, confirmed cone map.
Classical PCL pipeline, no machine learning.

`cone_detector_node`'s raw per-scan output (`/cones/observed`) is what
actually feeds the live planning/control loop (see
[`cone_planner`](../cone_planner/README.md)); `cone_mapper_node`'s confirmed
map (`/cones/map`) is not currently consumed downstream — it exists today for
visualization and as a stepping stone toward map-based planning.

## `cone_detector_node` pipeline

```
/points (PointCloud2)
   -> CropBox        (region of interest, tested in the LiDAR's own frame --
                       see note below; z bounds remove ground + ceiling)
   -> transform survivors into target_frame (default base_link)
   -> VoxelGrid      (downsample)
   -> Euclidean clustering
   -> shape filter   (keep cup-sized clusters)
   -> centroids
/cones/observed (geometry_msgs/PoseArray, in target_frame)   <- the data
/cones/markers  (visualization_msgs/MarkerArray)  <- for RViz
/debug/cropped, /debug/clusters (PointCloud2)     <- tuning aids
```

**Crop before transform, on purpose.** The obvious order is transform-then-crop,
but `tf2::doTransform` on a full raw `PointCloud2` (~86k points for the RSAIRY)
measured ~85-90ms/frame on the deploy laptop -- the dominant cost in the whole
callback. Instead, `pcl::CropBox::setTransform()` is given the sensor->target
transform: it uses that transform only to test each point against the ROI
bounds, and returns the untransformed points at the surviving indices (see
`pcl/filters/crop_box.h`). Only that small surviving subset then gets the real
`pcl::transformPointCloud` call. Same geometry, ~30-60x fewer points touched by
the actual SE3 transform. If you ever need to change how/where the transform
happens, keep this ordering in mind -- reverting to transform-then-crop is a
straightforward change but reintroduces that cost.

### Configuration (`config/params.yaml`)

| Key | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to transform the cloud into before cropping/clustering |
| `crop_min/max_x` | `0.2–2.5 m` | Forward region of interest |
| `crop_min/max_y` | `±1.0 m` | Lateral region of interest |
| `crop_min/max_z` | `0.08–0.40 m` | z bounds for ground removal |
| `voxel_leaf` | `0.02 m` | Downsample resolution before clustering; `<= 0` disables it. Caps clustering cost on dense/large point clumps in the ROI — see the note above the value in `params.yaml` for why this is on |
| `cluster_tolerance` | `0.12 m` | Max gap within a cluster |
| `min/max_cluster_size` | `2–3000` | Point count gates |
| `min_top_z` | `0.14 m` | Cluster must reach above this height to count as a cone |
| `min/max_footprint` | `0.0–0.20 m` | Accepted cluster width/depth |
| `min/max_height` | `0.02–0.30 m` | Accepted cluster height |
| `max_aspect` | `10.0` | Footprint/min-dimension gate; effectively off by default (bearing-dependent, was rejecting real cones) |
| `diagnostic` | `true` | Log every cluster's dimensions + a per-stage `timing:` line (transform/crop/voxel/cluster/shape/total) every 5th frame, and show labelled boxes on `/debug/cluster_info` |
| `publish_debug` | `true` | Publish `/debug/*` topics |

## `cone_mapper_node`

Tracks `/cones/observed` over time into a persistent, confirmed cone map —
smoothing out the frame-to-frame flicker of a single scan.

```
/cones/observed (PoseArray, target_frame)
   -> associate each detection with an existing track within association_radius
      (or start a new track if none matches)
   -> a track is CONFIRMED once it has min_observations hits
   -> tracks with no hit in forget_unconfirmed_sec are dropped (unconfirmed only)
/cones/map (PoseArray)                  <- confirmed cones only
/cones/map_markers (MarkerArray)        <- for RViz
```

### Configuration (`config/params.yaml`)

| Key | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame the map lives in |
| `association_radius` | `0.20 m` | Distance to match a detection to an existing tracked cone |
| `min_observations` | `5` | Detections needed before a cone is confirmed and published to `/cones/map` |
| `forget_unconfirmed_sec` | `2.0 s` | Unconfirmed tracks older than this are dropped |

## Build

Put this folder in a ROS 2 workspace and build:

```bash
mkdir -p ~/ros2_ws/src
cp -r cone_detector ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # pulls PCL, pcl_conversions
colcon build --packages-select cone_detector
source install/setup.bash
```

## Run

The recommended workflow is to develop against a recorded bag, not live hardware.

1. Bring up your LiDAR driver so `/points` is publishing, then record a drive
   through a field of cups:

   ```bash
   ros2 bag record /points /tf /tf_static -o cones_bag
   ```

2. In one terminal, launch the detector (this also starts the `base_link` ->
   LiDAR static TF and `cone_mapper_node`):

   ```bash
   ros2 launch cone_detector cone_detector.launch.py
   ```

   If your LiDAR publishes on a topic other than `/points`, either edit the
   remapping in the launch file or pass it on the command line:

   ```bash
   ros2 run cone_detector cone_detector_node --ros-args -r /points:=/rslidar_points
   ```

3. In another terminal, replay the bag (loop for easy tuning):

   ```bash
   ros2 bag play cones_bag --loop
   ```

4. Open RViz and add displays for:
   - `/points` (PointCloud2) — the raw cloud
   - `/debug/cropped` (PointCloud2) — after crop + downsample
   - `/debug/clusters` (PointCloud2) — only points that passed the shape filter
   - `/cones/markers` (MarkerArray) — the detected cones as cylinders, per scan
   - `/cones/map_markers` (MarkerArray) — confirmed, persistent cone map from `cone_mapper_node`
   Set the RViz fixed frame to `base_link` (or your LiDAR frame if not using
   the static TF).

## Tuning (do this stage by stage, watching RViz)

Edit `config/params.yaml` and re-launch — no recompile needed.

1. **Crop first.** Adjust `crop_*` until `/debug/cropped` shows only the floor
   region in front of the car, with ground and ceiling gone. The z bounds are
   relative to the LiDAR; account for your mount height (see the note in the
   yaml). Getting the z floor right IS your ground removal.
2. **Voxel leaf.** If distant cups disappear, lower `voxel_leaf`. If it's slow,
   raise it. ~0.02 m is a good start for 9 cm cups.
3. **Clustering.** If one cup splits into two clusters, raise `cluster_tolerance`.
   If two nearby cups merge, lower it. `min_cluster_size` sets your effective
   detection range (distant cups have fewer points).
4. **Shape filter.** Watch `/debug/clusters`. If walls/legs leak through, tighten
   `max_footprint` / `max_height`. If real cups get rejected, loosen the bounds.
5. **Keeping up in real time.** With `diagnostic: true`, watch the console for
   the `timing:` line (every 5th frame): raw/cropped/clustered point counts and
   a per-stage breakdown (`transform`/`nan`/`crop`/`voxel`/`cluster`/`shape`/
   `TOTAL`, ms). If `TOTAL` creeps toward your LiDAR's frame period, check which
   stage grew — a large non-cone object sitting in the ROI (rejected but still
   fully clustered before being thrown away) is the usual culprit, and a
   smaller/larger `voxel_leaf` is the first lever to reach for.

## Definition of done

Replaying the bag: every cup in range shows as exactly one marker, the markers
sit where the cups actually are, and walls/legs/noise produce no markers. Lay
out a known pattern of N cups and count detections vs. false positives — that
precision/recall number is both your quality gate and a metric for your report.

## Notes / upgrade paths

- Ground removal here is a simple z-threshold via the crop box, which assumes a
  flat floor that is level relative to the sensor. If the LiDAR is tilted, swap
  in RANSAC plane segmentation (`pcl::SACSegmentation`, `SACMODEL_PLANE`) before
  clustering.
- Parameters are read once at startup. To re-tune live without re-launching,
  add an `on_set_parameters` callback.
