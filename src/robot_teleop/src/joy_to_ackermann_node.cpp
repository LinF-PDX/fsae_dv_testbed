// joy_to_ackermann_node.cpp
//
// Translates an Xbox-style gamepad (/joy) into car-like drive commands
// (/cmd, ackermann_msgs/AckermannDriveStamped) for the tricycle robot.
//
// Design notes:
//  - Publishes on a fixed timer (default 50 Hz) from the latest joystick
//    state, NOT only on joy callbacks. This gives the firmware a steady
//    command stream to feed its watchdog, and lets us emit a safe zero
//    command the instant the controller goes quiet.
//  - Deadman button: speed is zero unless a chosen button is held. Let go
//    and the robot stops. This is the software half of "don't hit the wall";
//    the firmware watchdog is the hardware half.
//  - All axis/button indices, scales, and inversions are parameters, because
//    the "correct" mapping depends on the controller and on which direction
//    feels right -- you will almost certainly flip at least one invert flag.

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class JoyToAckermann : public rclcpp::Node
{
public:
  JoyToAckermann() : Node("joy_to_ackermann")
  {
    speed_axis_      = declare_parameter<int>("speed_axis", 1);       // left stick vertical
    steering_axis_   = declare_parameter<int>("steering_axis", 3);    // right stick horizontal
    deadman_button_  = declare_parameter<int>("deadman_button", 5);   // RB; -1 disables deadman
    max_speed_       = declare_parameter<double>("max_speed", 1.0);   // m/s at full stick
    max_steering_    = declare_parameter<double>("max_steering_angle", 0.5); // rad at full stick
    invert_speed_    = declare_parameter<bool>("invert_speed", true);  // Xbox: stick up = -1
    invert_steering_ = declare_parameter<bool>("invert_steering", false);
    publish_rate_    = declare_parameter<double>("publish_rate", 50.0);
    joy_timeout_     = declare_parameter<double>("joy_timeout", 0.5);  // s; no joy -> stop
    frame_id_        = declare_parameter<std::string>("frame_id", "base_link");

    pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToAckermann::onJoy, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JoyToAckermann::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "joy_to_ackermann up. Hold button %d (deadman) to drive. "
      "max_speed=%.2f m/s, max_steer=%.2f rad.",
      deadman_button_, max_speed_, max_steering_);
  }

private:
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = msg;
    last_joy_time_ = now();
  }

  void onTimer()
  {
    double speed = 0.0;
    double steering = 0.0;

    const bool fresh = last_joy_ &&
      (now() - last_joy_time_).seconds() < joy_timeout_;

    if (fresh) {
      const auto & j = *last_joy_;
      const bool deadman_held =
        (deadman_button_ < 0) ||  // -1 means deadman disabled
        (deadman_button_ < static_cast<int>(j.buttons.size()) &&
         j.buttons[deadman_button_] == 1);

      if (deadman_held) {
        double s = axis(j, speed_axis_);
        double t = axis(j, steering_axis_);
        if (invert_speed_)    s = -s;
        if (invert_steering_) t = -t;
        speed    = (-s + 1) * 0.5 * max_speed_;
        steering = t * max_steering_;
      }
    }
    // else: stale/no joy -> publish zero (safe stop), still at full rate.

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.drive.speed = speed;
    cmd.drive.steering_angle = steering;
    pub_->publish(cmd);
  }

  static double axis(const sensor_msgs::msg::Joy & j, int idx)
  {
    if (idx < 0 || idx >= static_cast<int>(j.axes.size())) return 0.0;
    return j.axes[idx];
  }

  int speed_axis_, steering_axis_, deadman_button_;
  double max_speed_, max_steering_, publish_rate_, joy_timeout_;
  bool invert_speed_, invert_steering_;
  std::string frame_id_;

  sensor_msgs::msg::Joy::SharedPtr last_joy_;
  rclcpp::Time last_joy_time_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToAckermann>());
  rclcpp::shutdown();
  return 0;
}
