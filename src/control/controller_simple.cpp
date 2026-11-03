#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mtrackercar
{

using namespace std::chrono_literals;

class ControllerSimple : public rclcpp::Node
{
public:
  ControllerSimple()
  : Node("mtrackercar_controller_simple"),
    enabled_(declare_parameter<bool>("enable_motion", false)),
    linear_velocity_(declare_parameter<double>("linear_velocity", 0.05)),
    angular_velocity_(declare_parameter<double>("angular_velocity", 0.2))
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(20ms, [this]() {publish_command();});

    if (enabled_) {
      RCLCPP_WARN(
        get_logger(), "Demo motion enabled: linear=%.3f m/s, angular=%.3f rad/s",
        linear_velocity_, angular_velocity_);
    } else {
      RCLCPP_INFO(
        get_logger(), "Demo motion disabled; set enable_motion:=true to publish motion commands");
    }
  }

  ~ControllerSimple() override
  {
    publisher_->publish(geometry_msgs::msg::Twist{});
  }

private:
  void publish_command()
  {
    geometry_msgs::msg::Twist command;
    if (enabled_) {
      command.linear.x = linear_velocity_;
      command.angular.z = angular_velocity_;
    }
    publisher_->publish(command);
  }

  const bool enabled_;
  const double linear_velocity_;
  const double angular_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtrackercar

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtrackercar::ControllerSimple>());
  rclcpp::shutdown();
  return 0;
}
