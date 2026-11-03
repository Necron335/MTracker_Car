#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class MtJoy : public rclcpp::Node
{
public:
  MtJoy() : Node("mtrackercar_joy")
  {
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    max_speed_ = declare_parameter<double>("max_speed", 0.5);
    max_steering_ = declare_parameter<double>("max_steering_angle", 0.26);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.190);
    if (!std::isfinite(max_speed_) || max_speed_ <= 0 ||
      !std::isfinite(max_steering_) || max_steering_ <= 0 ||
      max_steering_ >= 1.5707963267948966 ||
      !std::isfinite(wheel_base_) || wheel_base_ <= 0)
    {
      throw std::invalid_argument("Invalid joystick speed, steering limit or wheelbase");
    }
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_joy(*msg);});
  }

private:
  void on_joy(const sensor_msgs::msg::Joy & joy)
  {
    geometry_msgs::msg::Twist command;
    if (joy.axes.size() < 2 || joy.buttons.size() < 4 ||
      !std::isfinite(joy.axes[0]) || !std::isfinite(joy.axes[1]))
    {
      mode_ = 1;
      publisher_->publish(command);
      return;
    }
    // Preserve the supplied controller's latched modes and button priority.
    if (joy.buttons[0]) mode_ = 0;
    else if (joy.buttons[1]) mode_ = 1;
    else if (joy.buttons[2]) mode_ = 2;
    else if (joy.buttons[3]) mode_ = 3;

    double speed = 0, steering = 0;
    if (mode_ == 0) {
      speed = std::clamp(static_cast<double>(joy.axes[1]), -max_speed_, max_speed_);
      steering = std::clamp(static_cast<double>(joy.axes[0]), -max_steering_, max_steering_);
    } else if (mode_ == 2 || mode_ == 3) {
      speed = max_speed_;
      steering = mode_ == 2 ? max_steering_ : -max_steering_;
    }
    if (enable_motion_) {
      command.linear.x = speed;
      command.angular.z = speed * std::tan(steering) / wheel_base_;
    }
    publisher_->publish(command);
  }

  bool enable_motion_{};
  double max_speed_{}, max_steering_{}, wheel_base_{};
  int mode_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MtJoy>());
  rclcpp::shutdown();
  return 0;
}
