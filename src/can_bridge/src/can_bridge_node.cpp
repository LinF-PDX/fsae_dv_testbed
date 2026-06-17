// can_bridge_node.cpp
//
// Bridges between ROS (AckermannDriveStamped on /cmd) and the STM32
// via ros2_socketcan (can_msgs/Frame on /to_can_bus and /from_can_bus).
//
// Outbound:  /cmd -> pack DV_Command (ID 0x200) -> /to_can_bus
// Inbound:   /from_can_bus -> unpack DV_Feedback (ID 0x201) -> /encoder
//
// All CAN IDs, signal layouts, scaling, and units are parameters so the
// DBC can change without recompiling. The speed conversion (m/s -> PWM)
// is a single configurable ratio.
//
// Switching input source (teleop vs autonomy): the bridge listens on one
// topic (default /cmd). Both teleop and the future controller publish
// there. To add a proper mux later, change cmd_topic to /cmd_mux/output
// and route teleop + controller through the mux.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/int32.hpp>

class CanBridge : public rclcpp::Node
{
public:
  CanBridge() : Node("can_bridge")
  {
    // --- CAN IDs (hex in DBC, decimal here) ---
    cmd_can_id_  = declare_parameter<int>("cmd_can_id", 0x200);   // DV_Command
    fb_can_id_   = declare_parameter<int>("fb_can_id", 0x201);    // DV_Feedback

    // --- Scaling ---
    // Steering: ROS is radians, DBC is degrees * 10 (0.1 deg resolution).
    // rad -> deg: * 180/pi.  deg -> DBC: * 10.  Combined: * 1800/pi ≈ 572.96
    steering_scale_ = declare_parameter<double>("steering_scale", 1800.0 / M_PI);

    // Speed: ROS is m/s, DBC is raw PWM (0-1000).
    // User calibrates: at speed_at_max_pwm m/s the firmware gets max_pwm.
    // PWM = speed * (max_pwm / speed_at_max_pwm)
    max_pwm_            = declare_parameter<double>("max_pwm", 1000.0);
    speed_at_max_pwm_   = declare_parameter<double>("speed_at_max_pwm", 0.2);

    // Clamps applied AFTER conversion, in DBC units.
    max_steering_dbc_  = declare_parameter<double>("max_steering_dbc", 300.0);  // ±30 deg
    max_speed_pwm_     = declare_parameter<double>("max_speed_pwm", 1000.0);

    // --- Topics ---
    std::string cmd_topic = declare_parameter<std::string>("cmd_topic", "/cmd");

    cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      cmd_topic, 10,
      std::bind(&CanBridge::onCmd, this, std::placeholders::_1));

    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      "/from_can_bus", 100,
      std::bind(&CanBridge::onCanRx, this, std::placeholders::_1));

    can_pub_ = create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
    enc_pub_ = create_publisher<std_msgs::msg::Int32>("/encoder", 10);

    RCLCPP_INFO(get_logger(),
      "can_bridge up. cmd=0x%X fb=0x%X speed_scale=%.1f pwm/%.2f m/s, "
      "listening on %s",
      cmd_can_id_, fb_can_id_, max_pwm_, speed_at_max_pwm_,
      cmd_topic.c_str());
  }

private:
  // ── Outbound: /cmd -> CAN ──────────────────────────────────────────
  void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    const double steer_rad = msg->drive.steering_angle;
    const double speed_mps = msg->drive.speed;

    // Convert to DBC units.
    double steer_dbc = steer_rad * steering_scale_;  // rad -> deg*10
    double speed_pwm = speed_mps * (max_pwm_ / speed_at_max_pwm_);  // m/s -> pwm

    // Clamp.
    steer_dbc = std::clamp(steer_dbc, -max_steering_dbc_, max_steering_dbc_);
    speed_pwm = std::clamp(speed_pwm, -max_speed_pwm_, max_speed_pwm_);

    // Pack into CAN frame per DBC:
    //   TargetSteeringAngle: bits 0-15, signed int16, little-endian
    //   TargetMotorSpeed:    bits 16-31, unsigned int16, little-endian
    // (if you need signed speed for reverse, change the cast to int16_t)
    const int16_t  steer_raw = static_cast<int16_t>(std::round(steer_dbc));
    const uint16_t speed_raw = static_cast<uint16_t>(
      std::round(std::max(0.0, speed_pwm)));  // unsigned: no reverse yet

    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.id = static_cast<uint32_t>(cmd_can_id_);
    frame.dlc = 8;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.is_error = false;

    std::memset(frame.data.data(), 0, 8);
    std::memcpy(&frame.data[0], &steer_raw, 2);  // bytes 0-1, LE
    std::memcpy(&frame.data[2], &speed_raw, 2);  // bytes 2-3, LE

    can_pub_->publish(frame);
  }

  // ── Inbound: CAN -> /encoder ───────────────────────────────────────
  void onCanRx(const can_msgs::msg::Frame::SharedPtr msg)
  {
    if (static_cast<int>(msg->id) != fb_can_id_) return;
    if (msg->dlc < 4) return;

    // EncoderCount: bits 0-31, unsigned int32, little-endian.
    uint32_t raw = 0;
    std::memcpy(&raw, &msg->data[0], 4);

    std_msgs::msg::Int32 out;
    out.data = static_cast<int32_t>(raw);
    enc_pub_->publish(out);
  }

  // params
  int cmd_can_id_, fb_can_id_;
  double steering_scale_;
  double max_pwm_, speed_at_max_pwm_;
  double max_steering_dbc_, max_speed_pwm_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}
