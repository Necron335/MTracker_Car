#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mtrackercar
{

using namespace std::chrono_literals;

class SetOdometryDemo : public rclcpp::Node
{
public:
  SetOdometryDemo()
  : Node("set_odometry_demo"),
    x_(declare_parameter<double>("x", 0.0)),
    y_(declare_parameter<double>("y", 0.0)),
    theta_(declare_parameter<double>("theta", 0.0))
  {
    publisher_ = create_publisher<geometry_msgs::msg::Pose2D>("set_odometry", 1);
    timer_ = create_wall_timer(500ms, [this]() {handle_timer();});
    RCLCPP_INFO(
      get_logger(), "Will set odometry to x=%.3f, y=%.3f, theta=%.3f",
      x_, y_, theta_);
  }

private:
  void handle_timer()
  {
    if (!published_) {
      geometry_msgs::msg::Pose2D pose;
      pose.x = x_;
      pose.y = y_;
      pose.theta = theta_;
      publisher_->publish(pose);
      published_ = true;
      RCLCPP_INFO(get_logger(), "Odometry command published");
      return;
    }

    RCLCPP_INFO(get_logger(), "Demo completed");
    rclcpp::shutdown();
  }

  const double x_;
  const double y_;
  const double theta_;
  bool published_{false};
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtrackercar

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtrackercar::SetOdometryDemo>());
  return 0;
}
