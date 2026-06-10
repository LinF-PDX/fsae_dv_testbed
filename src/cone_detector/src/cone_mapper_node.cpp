// cone_mapper_node.cpp
//
// Subscribes to per-scan cone detections (/cones/observed, PoseArray), and
// maintains a persistent map of cones by temporal accumulation:
//   - each detection is associated to the nearest tracked cone within
//     association_radius, or starts a new track;
//   - a track's position is the running average of its observations;
//   - a track becomes CONFIRMED after min_observations hits;
//   - unconfirmed tracks that stop being seen are forgotten.
//
// Only confirmed cones are published (/cones/map + /cones/map_markers), so
// single-frame detection flicker and one-off false positives never reach the
// map. This is the standard FSD pattern: perception is noisy, the map is
// what you trust.
//
// Frames: detections arrive in the detector's frame (base_link for a static
// robot). target_frame defaults to base_link; once odometry exists, set it
// to "odom" and the map persists correctly while the robot drives.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct TrackedCone
{
  double x{0.0};
  double y{0.0};
  int hits{0};
  bool confirmed{false};
  int id{-1};
  rclcpp::Time last_seen;
};

class ConeMapper : public rclcpp::Node
{
public:
  ConeMapper() : Node("cone_mapper")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    association_radius_ = declare_parameter<double>("association_radius", 0.20);
    min_observations_ = declare_parameter<int>("min_observations", 5);
    // Unconfirmed tracks not seen for this long are dropped (transient noise).
    // Confirmed cones are never dropped (static track assumption).
    forget_unconfirmed_sec_ = declare_parameter<double>("forget_unconfirmed_sec", 2.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/cones/observed", 10,
      std::bind(&ConeMapper::onCones, this, std::placeholders::_1));

    map_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/cones/map", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cones/map_markers", 10);

    RCLCPP_INFO(get_logger(),
      "cone_mapper started: confirm after %d hits, associate within %.2f m, frame %s",
      min_observations_, association_radius_, target_frame_.c_str());
  }

private:
  void onCones(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Transform detections into the map frame (identity while both are
    // base_link; becomes meaningful once target_frame = odom).
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s unavailable: %s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const rclcpp::Time now = get_clock()->now();

    for (const auto & pose : msg->poses) {
      geometry_msgs::msg::PoseStamped in, out;
      in.header = msg->header;
      in.pose = pose;
      tf2::doTransform(in, out, tf);
      integrate(out.pose.position.x, out.pose.position.y, now);
    }

    pruneStale(now);
    publishMap();
  }

  void integrate(double x, double y, const rclcpp::Time & now)
  {
    // Nearest-neighbour association.
    TrackedCone * best = nullptr;
    double best_d2 = association_radius_ * association_radius_;
    for (auto & c : cones_) {
      const double dx = c.x - x;
      const double dy = c.y - y;
      const double d2 = dx * dx + dy * dy;
      if (d2 <= best_d2) {
        best_d2 = d2;
        best = &c;
      }
    }

    if (best != nullptr) {
      // Running average over all hits: stable, converges, order-independent.
      const double k = static_cast<double>(best->hits);
      best->x = (best->x * k + x) / (k + 1.0);
      best->y = (best->y * k + y) / (k + 1.0);
      best->hits += 1;
      best->last_seen = now;
      if (!best->confirmed && best->hits >= min_observations_) {
        best->confirmed = true;
        best->id = next_id_++;
        RCLCPP_INFO(get_logger(), "cone #%d confirmed at (%.2f, %.2f) after %d hits",
                    best->id, best->x, best->y, best->hits);
      }
    } else {
      TrackedCone c;
      c.x = x;
      c.y = y;
      c.hits = 1;
      c.last_seen = now;
      cones_.push_back(c);
    }
  }

  void pruneStale(const rclcpp::Time & now)
  {
    if (forget_unconfirmed_sec_ <= 0.0) return;
    cones_.erase(
      std::remove_if(cones_.begin(), cones_.end(),
        [&](const TrackedCone & c) {
          return !c.confirmed &&
                 (now - c.last_seen).seconds() > forget_unconfirmed_sec_;
        }),
      cones_.end());
  }

  void publishMap()
  {
    geometry_msgs::msg::PoseArray map;
    map.header.frame_id = target_frame_;
    map.header.stamp = get_clock()->now();

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = map.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    for (const auto & c : cones_) {
      if (!c.confirmed) continue;

      geometry_msgs::msg::Pose p;
      p.position.x = c.x;
      p.position.y = c.y;
      p.position.z = 0.12;  // marker centre at half cone height
      p.orientation.w = 1.0;
      map.poses.push_back(p);

      visualization_msgs::msg::Marker m;
      m.header = map.header;
      m.ns = "map_cones";
      m.id = c.id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = p;
      m.scale.x = 0.08;
      m.scale.y = 0.08;
      m.scale.z = 0.24;
      m.color.r = 0.1f; m.color.g = 0.9f; m.color.b = 0.2f; m.color.a = 0.95f;
      // lifetime left at 0 (forever): markers persist, no blinking. DELETEALL
      // at the top of each publish keeps the set in sync.
      arr.markers.push_back(m);
    }

    map_pub_->publish(map);
    marker_pub_->publish(arr);
  }

  // params
  std::string target_frame_;
  double association_radius_;
  int min_observations_;
  double forget_unconfirmed_sec_;

  std::vector<TrackedCone> cones_;
  int next_id_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMapper>());
  rclcpp::shutdown();
  return 0;
}