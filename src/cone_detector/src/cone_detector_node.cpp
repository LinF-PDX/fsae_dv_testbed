// cone_detector_node.cpp
//
// Subscribes to a LiDAR PointCloud2, runs a classical PCL pipeline
// (crop -> downsample -> cluster -> shape-filter), and publishes the
// detected cone centroids as a PoseArray plus a MarkerArray for RViz.
//
// All tunables are ROS parameters (see config/params.yaml) so you can
// adjust thresholds without recompiling.

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>  // transformToEigen

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>       // getMinMax3D
#include <pcl/common/centroid.h>     // compute3DCentroid
#include <pcl/common/transforms.h>   // transformPointCloud

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class ConeDetector : public rclcpp::Node
{
public:
  ConeDetector() : Node("cone_detector")
  {
    // --- Region of interest, in target_frame (default base_link, z = up). ---
    // The incoming cloud is transformed into target_frame first, so these
    // bounds mean what they say regardless of how the LiDAR is mounted.
    // With the LiDAR ~0.18 m up, the floor is at z = 0, so crop_min_z = 0.02
    // drops the ground and crop_max_z = 0.13 keeps the tops of ~9 cm cups.
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    crop_min_x_ = declare_parameter<double>("crop_min_x", 0.0);
    crop_max_x_ = declare_parameter<double>("crop_max_x", 6.0);
    crop_min_y_ = declare_parameter<double>("crop_min_y", -3.0);
    crop_max_y_ = declare_parameter<double>("crop_max_y", 3.0);
    crop_min_z_ = declare_parameter<double>("crop_min_z", 0.02);
    crop_max_z_ = declare_parameter<double>("crop_max_z", 0.13);

    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.02);

    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.05);
    min_cluster_size_  = declare_parameter<int>("min_cluster_size", 3);
    max_cluster_size_  = declare_parameter<int>("max_cluster_size", 300);

    // Shape gate. min_top_z (highest point of cluster above floor) is the main
    // discriminator for tall cones vs leaked ground; extent gates are secondary.
    min_footprint_ = declare_parameter<double>("min_footprint", 0.0);
    max_footprint_ = declare_parameter<double>("max_footprint", 0.20);
    min_height_    = declare_parameter<double>("min_height", 0.02);
    max_height_    = declare_parameter<double>("max_height", 0.40);
    min_top_z_     = declare_parameter<double>("min_top_z", 0.14);
    max_aspect_    = declare_parameter<double>("max_aspect", 10.0);

    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    diagnostic_    = declare_parameter<bool>("diagnostic", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", sensor_qos,
      std::bind(&ConeDetector::onCloud, this, std::placeholders::_1));

    cones_pub_  = create_publisher<geometry_msgs::msg::PoseArray>("/cones/observed", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cones/markers", 10);
    dbg_info_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/cluster_info", 10);

    if (publish_debug_) {
      dbg_crop_pub_     = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/cropped", 1);
      dbg_clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/clusters", 1);
    }

    RCLCPP_INFO(get_logger(), "cone_detector started; waiting for /points ...");
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    using clock = std::chrono::steady_clock;
    const auto t_start = clock::now();

    // --- 0. Look up the (static) sensor -> target_frame transform. ---
    // We do NOT transform the whole cloud here: tf2::doTransform on a raw
    // PointCloud2 walks every field of every point through the generic ROS
    // message iterator, which measured ~85-90ms on the full ~86k-point RSAIRY
    // frame -- the dominant cost in this callback (see timing log below).
    // Instead we crop first, in the sensor's own frame, using PCL's CropBox
    // with its `transform` set to sensor->target_frame: it transforms each
    // point ONLY for the bounds test (see crop_box.hpp) and returns the
    // ORIGINAL, still-sensor-frame points. Only the small surviving subset
    // is then explicitly transformed via pcl::transformPointCloud. Net
    // effect: identical geometry, ~30-60x fewer points touched by the
    // (relatively expensive) SE3 transform.
    Eigen::Affine3f sensor_to_target;
    try {
      auto tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      sensor_to_target = Eigen::Affine3f(
        tf2::transformToEigen(tf).matrix().cast<float>());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s not available yet: %s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }
    const auto t_transform = clock::now();
    const size_t raw_points = static_cast<size_t>(msg->width) * msg->height;

    std_msgs::msg::Header header_out;
    header_out.frame_id = target_frame_;
    header_out.stamp = msg->header.stamp;

    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    // rslidar may emit NaN-padded (non-dense) clouds; CropBox already skips
    // non-finite points internally when is_dense is false, so make sure that
    // flag reflects reality rather than trusting whatever fromROSMsg guessed.
    cloud->is_dense = false;
    if (cloud->empty()) return;
    const auto t_nan = clock::now();

    // --- 1. Crop to ROI, in the sensor's own frame (also removes ground +
    // ceiling via z bounds once transformed). ---
    CloudT::Ptr cropped_sensor_frame(new CloudT);
    {
      pcl::CropBox<PointT> crop;
      crop.setInputCloud(cloud);
      crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0f));
      crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0f));
      crop.setTransform(sensor_to_target);
      crop.filter(*cropped_sensor_frame);
    }
    if (cropped_sensor_frame->empty()) return;

    // Now transform just the (small) surviving subset into target_frame --
    // this is the only per-point SE3 transform we pay for.
    CloudT::Ptr cropped(new CloudT);
    pcl::transformPointCloud(*cropped_sensor_frame, *cropped, sensor_to_target);
    cropped->is_dense = true;  // CropBox already dropped non-finite points
    const auto t_crop = clock::now();

    // --- 2. Optional voxel downsample. voxel_leaf <= 0 disables it. ---
    // Downsampling thins far cones (already few points), so for this cloud
    // size on a laptop we default to OFF and cluster the raw cropped cloud.
    CloudT::Ptr ds;
    if (voxel_leaf_ > 0.0) {
      ds.reset(new CloudT);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cropped);
      vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
      vg.filter(*ds);
    } else {
      ds = cropped;
    }
    if (ds->empty()) return;
    const auto t_voxel = clock::now();

    if (publish_debug_ && dbg_crop_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*ds, out);
      out.header = header_out;
      dbg_crop_pub_->publish(out);
    }

    // --- 3. Euclidean clustering ---
    std::vector<pcl::PointIndices> clusters;
    {
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree->setInputCloud(ds);
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(ds);
      ec.extract(clusters);
    }
    const auto t_cluster = clock::now();

    // --- 4. Shape filter + centroids ---
    geometry_msgs::msg::PoseArray cones;
    cones.header = header_out;  // now in target_frame

    CloudT::Ptr accepted_pts(new CloudT);  // for debug viz
    const bool diag_now = diagnostic_ && (++frame_count_ % 5 == 0);
    visualization_msgs::msg::MarkerArray info;
    {
      visualization_msgs::msg::Marker clr;
      clr.header = header_out;
      clr.action = visualization_msgs::msg::Marker::DELETEALL;
      info.markers.push_back(clr);
    }
    int info_id = 0;

    for (const auto & cl : clusters) {
      CloudT::Ptr sub(new CloudT);
      sub->reserve(cl.indices.size());
      for (int idx : cl.indices) sub->push_back((*ds)[idx]);

      Eigen::Vector4f min_pt, max_pt, centroid;
      pcl::getMinMax3D(*sub, min_pt, max_pt);
      pcl::compute3DCentroid(*sub, centroid);

      const int    n      = static_cast<int>(cl.indices.size());
      const double width  = max_pt.x() - min_pt.x();
      const double depth  = max_pt.y() - min_pt.y();
      const double height = max_pt.z() - min_pt.z();
      const double top    = max_pt.z();  // highest point above floor
      const double footprint = std::max(width, depth);
      const double min_dim   = std::max(1e-3, std::min(width, depth));
      const double aspect    = footprint / min_dim;

      // The top-z gate is the strongest discriminator: a tall cone's points sit
      // HIGH even when only 2 beams clip it (tiny extent), while leaked ground
      // sits just above the crop floor. Gate on where points ARE, not extent.
      const bool footprint_ok = footprint >= min_footprint_ && footprint <= max_footprint_;
      const bool height_ok    = height    >= min_height_    && height    <= max_height_;
      const bool top_ok       = top       >= min_top_z_;
      const bool aspect_ok    = aspect     <= max_aspect_;
      const bool accept = footprint_ok && height_ok && top_ok && aspect_ok;

      // Why did it fail? (first failing gate)
      const char * reason = "ACCEPT";
      if      (!footprint_ok) reason = "foot";
      else if (!top_ok)       reason = "top";
      else if (!height_ok)    reason = "height";
      else if (!aspect_ok)    reason = "aspect";

      if (diag_now) {
        RCLCPP_INFO(get_logger(),
          "cluster @(%.2f,%.2f) n=%d w=%.3f d=%.3f h=%.3f top=%.3f foot=%.3f asp=%.1f -> %s",
          centroid.x(), centroid.y(), n, width, depth, height, top, footprint, aspect, reason);
        addInfoMarkers(info, info_id, header_out, centroid,
                       width, depth, height,
                       n, top, aspect, accept, reason);
      }

      if (!accept) continue;

      geometry_msgs::msg::Pose p;
      p.position.x = centroid.x();
      p.position.y = centroid.y();
      p.position.z = centroid.z();
      p.orientation.w = 1.0;  // no orientation for a cone
      cones.poses.push_back(p);

      if (publish_debug_) *accepted_pts += *sub;
    }

    if (diag_now) {
      RCLCPP_INFO(get_logger(), "--- %zu clusters, %zu accepted ---",
                  clusters.size(), cones.poses.size());
      dbg_info_pub_->publish(info);

      const auto t_end = clock::now();
      auto ms = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };
      RCLCPP_INFO(get_logger(),
        "timing: raw=%zu pts, cropped=%zu, clustered=%zu pts | "
        "transform=%.1fms nan=%.1fms crop=%.1fms voxel=%.1fms cluster=%.1fms "
        "shape=%.1fms TOTAL=%.1fms",
        raw_points, cropped->size(), ds->size(),
        ms(t_start, t_transform), ms(t_transform, t_nan), ms(t_nan, t_crop),
        ms(t_crop, t_voxel), ms(t_voxel, t_cluster), ms(t_cluster, t_end),
        ms(t_start, t_end));
    }

    cones_pub_->publish(cones);
    publishMarkers(cones);

    if (publish_debug_ && dbg_clusters_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*accepted_pts, out);
      out.header = header_out;
      dbg_clusters_pub_->publish(out);
    }

    RCLCPP_DEBUG(get_logger(), "clusters=%zu accepted=%zu",
                 clusters.size(), cones.poses.size());
  }

  void publishMarkers(const geometry_msgs::msg::PoseArray & cones)
  {
    visualization_msgs::msg::MarkerArray arr;

    // Clear last frame's markers first.
    visualization_msgs::msg::Marker clear;
    clear.header = cones.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto & pose : cones.poses) {
      visualization_msgs::msg::Marker m;
      m.header = cones.header;
      m.ns = "cones";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = 0.09;  // diameter
      m.scale.y = 0.09;
      m.scale.z = 0.09;  // height
      m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; m.color.a = 0.9f;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  // Per-cluster diagnostic markers: a wireframe-ish box at the cluster's bounding
  // box plus a text label showing n / height / aspect. Green = accepted, red = rejected.
  void addInfoMarkers(visualization_msgs::msg::MarkerArray & arr, int & id,
                      const std_msgs::msg::Header & header,
                      const Eigen::Vector4f & centroid,
                      double width, double depth, double height,
                      int n, double h, double aspect, bool accept,
                      const char * reason)
  {
    visualization_msgs::msg::Marker box;
    box.header = header;
    box.ns = "cluster_box";
    box.id = id;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = centroid.x();
    box.pose.position.y = centroid.y();
    box.pose.position.z = centroid.z();
    box.pose.orientation.w = 1.0;
    box.scale.x = std::max(width,  0.01);
    box.scale.y = std::max(depth,  0.01);
    box.scale.z = std::max(height, 0.01);
    box.color.r = accept ? 0.0f : 1.0f;
    box.color.g = accept ? 1.0f : 0.0f;
    box.color.b = 0.0f;
    box.color.a = 0.35f;
    box.lifetime = rclcpp::Duration::from_seconds(0.5);
    arr.markers.push_back(box);

    visualization_msgs::msg::Marker txt;
    txt.header = header;
    txt.ns = "cluster_text";
    txt.id = id;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = centroid.x();
    txt.pose.position.y = centroid.y();
    txt.pose.position.z = centroid.z() + 0.10;
    txt.pose.orientation.w = 1.0;
    txt.scale.z = 0.04;  // text height in metres
    txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "n=%d t=%.2f a=%.1f %s", n, h, aspect, reason);
    txt.text = buf;
    txt.lifetime = rclcpp::Duration::from_seconds(0.5);
    arr.markers.push_back(txt);

    ++id;
  }
  std::string target_frame_;
  double crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_, crop_min_z_, crop_max_z_;
  double voxel_leaf_;
  double cluster_tolerance_;
  int min_cluster_size_, max_cluster_size_;
  double min_footprint_, max_footprint_, min_height_, max_height_, min_top_z_;
  double max_aspect_;
  bool publish_debug_;
  bool diagnostic_;
  uint64_t frame_count_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cones_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbg_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dbg_crop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dbg_clusters_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetector>());
  rclcpp::shutdown();
  return 0;
}