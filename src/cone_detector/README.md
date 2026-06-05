# cone_detector

A ROS 2 node that detects cones (inverted cups) in a LiDAR point cloud and
publishes their 2D positions. Classical PCL pipeline, no machine learning.

## Pipeline

```
/points (PointCloud2)
   -> CropBox        (region of interest; z bounds remove ground + ceiling)
   -> VoxelGrid      (downsample)
   -> Euclidean clustering
   -> shape filter   (keep cup-sized clusters)
   -> centroids
/cones/observed (geometry_msgs/PoseArray)   <- the data
/cones/markers  (visualization_msgs/MarkerArray)  <- for RViz
/debug/cropped, /debug/clusters (PointCloud2)     <- tuning aids
```

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

2. In one terminal, launch the detector:

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
   - `/cones/markers` (MarkerArray) — the detected cones as cylinders
   Set the RViz fixed frame to your LiDAR frame (the `frame_id` on `/points`).

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

## Definition of done

Replaying the bag: every cup in range shows as exactly one marker, the markers
sit where the cups actually are, and walls/legs/noise produce no markers. Lay
out a known pattern of N cups and count detections vs. false positives — that
precision/recall number is both your quality gate and a metric for your report.

## Notes / upgrade paths

- Output is in the cloud's own frame (`frame_id` copied from `/points`). The
  downstream mapper node is the right place to transform into `odom`/`base_link`.
- Ground removal here is a simple z-threshold via the crop box, which assumes a
  flat floor that is level relative to the sensor. If the LiDAR is tilted, swap
  in RANSAC plane segmentation (`pcl::SACSegmentation`, `SACMODEL_PLANE`) before
  clustering.
- Parameters are read once at startup. To re-tune live without re-launching,
  add an `on_set_parameters` callback.
