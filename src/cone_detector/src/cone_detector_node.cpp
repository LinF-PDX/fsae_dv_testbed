// cone_detector_node.cpp
//
// Subscribes to a LiDAR PointCloud2, runs a classical PCL pipeline
// (crop -> downsample -> cluster -> shape-filter), and publishes the
// detected cone centroids as a PoseArray plus a MarkerArray for RViz.
//
// All tunables are ROS parameters (see config/params.yaml) so you can
// adjust thresholds without recompiling.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  // doTransform for PointCloud2

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>     // getMinMax3D
#include <pcl/common/centroid.h>   // compute3DCentroid

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

    // Shape gate: a cup-sized cluster. Footprint = horizontal extent, height = vertical.
    min_footprint_ = declare_parameter<double>("min_footprint", 0.03);
    max_footprint_ = declare_parameter<double>("max_footprint", 0.15);
    min_height_    = declare_parameter<double>("min_height", 0.03);
    max_height_    = declare_parameter<double>("max_height", 0.15);

    publish_debug_ = declare_parameter<bool>("publish_debug", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", sensor_qos,
      std::bind(&ConeDetector::onCloud, this, std::placeholders::_1));

    cones_pub_  = create_publisher<geometry_msgs::msg::PoseArray>("/cones/observed", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cones/markers", 10);

    if (publish_debug_) {
      dbg_crop_pub_     = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/cropped", 1);
      dbg_clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/clusters", 1);
    }

    RCLCPP_INFO(get_logger(), "cone_detector started; waiting for /points ...");
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // --- 0. Transform the cloud into target_frame (z = up) ---
    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
      auto tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, cloud_tf, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s not available yet: %s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(cloud_tf, *cloud);
    if (cloud->empty()) return;

    // --- 1. Crop to ROI (also removes ground + ceiling via z bounds) ---
    CloudT::Ptr cropped(new CloudT);
    {
      pcl::CropBox<PointT> crop;
      crop.setInputCloud(cloud);
      crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0f));
      crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0f));
      crop.filter(*cropped);
    }
    if (cropped->empty()) return;

    // --- 2. Voxel-grid downsample (evens out density, speeds up clustering) ---
    CloudT::Ptr ds(new CloudT);
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cropped);
      vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
      vg.filter(*ds);
    }
    if (ds->empty()) return;

    if (publish_debug_ && dbg_crop_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*ds, out);
      out.header = cloud_tf.header;
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

    // --- 4. Shape filter + centroids ---
    geometry_msgs::msg::PoseArray cones;
    cones.header = cloud_tf.header;  // now in target_frame

    CloudT::Ptr accepted_pts(new CloudT);  // for debug viz

    for (const auto & cl : clusters) {
      CloudT::Ptr sub(new CloudT);
      sub->reserve(cl.indices.size());
      for (int idx : cl.indices) sub->push_back((*ds)[idx]);

      Eigen::Vector4f min_pt, max_pt, centroid;
      pcl::getMinMax3D(*sub, min_pt, max_pt);
      pcl::compute3DCentroid(*sub, centroid);

      const double width  = max_pt.x() - min_pt.x();
      const double depth  = max_pt.y() - min_pt.y();
      const double height = max_pt.z() - min_pt.z();
      const double footprint = std::max(width, depth);

      const bool footprint_ok = footprint >= min_footprint_ && footprint <= max_footprint_;
      const bool height_ok    = height    >= min_height_    && height    <= max_height_;
      if (!footprint_ok || !height_ok) continue;

      geometry_msgs::msg::Pose p;
      p.position.x = centroid.x();
      p.position.y = centroid.y();
      p.position.z = centroid.z();
      p.orientation.w = 1.0;  // no orientation for a cone
      cones.poses.push_back(p);

      if (publish_debug_) *accepted_pts += *sub;
    }

    cones_pub_->publish(cones);
    publishMarkers(cones);

    if (publish_debug_ && dbg_clusters_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*accepted_pts, out);
      out.header = cloud_tf.header;
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

  // params
  std::string target_frame_;
  double crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_, crop_min_z_, crop_max_z_;
  double voxel_leaf_;
  double cluster_tolerance_;
  int min_cluster_size_, max_cluster_size_;
  double min_footprint_, max_footprint_, min_height_, max_height_;
  bool publish_debug_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cones_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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