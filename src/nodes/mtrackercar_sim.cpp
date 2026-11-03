#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "mtrackercar/control/car_model.hpp"

namespace mtrackercar
{
class Simulator : public rclcpp::Node
{
public:
  Simulator() : Node("mtrackercar"), transform_broadcaster_(*this)
  {
    enable_drive_ = declare_parameter<bool>("enable_drive", true);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.190);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
    max_steering_ = declare_parameter<double>("max_steering_angle", 0.26);
    servo_angle_ = declare_parameter<double>("servo_angle", 1.0471975511965976);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    const double rate = declare_parameter<double>("command_rate_hz", 100.0);
    const auto topic = declare_parameter<std::string>("odometry_topic", "/wheel/odom");
    odom_frame_ = declare_parameter<std::string>(
      "odom_frame_id", declare_parameter<std::string>("odom_frame", "odom"));
    base_frame_ = declare_parameter<std::string>(
      "frame_id", declare_parameter<std::string>("base_frame", "base_link"));
    pose_sigma_x_ = declare_parameter<double>("pose_sigma_x", 0.01);
    pose_sigma_y_ = declare_parameter<double>("pose_sigma_y", 0.01);
    pose_sigma_theta_ = declare_parameter<double>("pose_sigma_theta", 0.02);
    twist_sigma_linear_ = declare_parameter<double>("twist_sigma_linear", 0.02);
    twist_sigma_angular_ = declare_parameter<double>("twist_sigma_angular", 0.03);
    for (double value : {wheel_base_, wheel_radius_, max_steering_, servo_angle_,
        command_timeout_, rate}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Simulator geometry, timeout and rate must be finite and positive");
      }
    }
    if (max_steering_ >= 1.5707963267948966 || servo_angle_ < max_steering_) {
      throw std::invalid_argument("Invalid steering range");
    }
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(topic, 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    previous_time_ = last_command_time_ = now();
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        const auto stamp = now();
        advance(stamp);
        input_ = car_input(msg->linear.x, msg->angular.z, wheel_base_,
          wheel_radius_, max_steering_, servo_angle_);
        last_command_time_ = stamp;
      });
    reset_subscription_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "set_odometry", 1, [this](geometry_msgs::msg::Pose2D::ConstSharedPtr msg) {
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->theta)) return;
        advance(now());
        state_ = {msg->x, msg->y, std::atan2(std::sin(msg->theta), std::cos(msg->theta))};
        input_ = {};
        speed_ = yaw_rate_ = 0;
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1 / rate)),
      [this]() {const auto stamp = now(); advance(stamp); publish(stamp);});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      diagnostic_msgs::msg::DiagnosticArray message;
      message.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "mtrackercar/simulation";
      status.hardware_id = "kinematic_model";
      status.level = status.OK;
      status.message = "Ideal bicycle simulation";
      message.status.push_back(status);
      diagnostics_publisher_->publish(message);
    });
  }

private:
  void advance(const rclcpp::Time & stamp)
  {
    const double dt = (stamp - previous_time_).seconds();
    if (dt < 0) {
      input_ = {};
      last_command_time_ = stamp;
    }
    const double remaining = command_timeout_ - (previous_time_ - last_command_time_).seconds();
    const double active_dt = enable_drive_ ? std::clamp(remaining, 0.0, std::max(0.0, dt)) : 0.0;
    const double speed = input_.rear_velocity * wheel_radius_;
    const double yaw = speed * std::tan(input_.servo * servo_angle_) / wheel_base_;
    integrate_car(state_, speed, yaw, active_dt);
    const bool active = enable_drive_ && (stamp - last_command_time_).seconds() <= command_timeout_;
    speed_ = active ? speed : 0;
    yaw_rate_ = active ? yaw : 0;
    previous_time_ = stamp;
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = state_.x;
    odometry.pose.pose.position.y = state_.y;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, state_.theta);
    odometry.pose.pose.orientation = tf2::toMsg(rotation);
    odometry.twist.twist.linear.x = speed_;
    odometry.twist.twist.angular.z = yaw_rate_;
    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[0] = pose_sigma_x_ * pose_sigma_x_;
    odometry.pose.covariance[7] = pose_sigma_y_ * pose_sigma_y_;
    odometry.pose.covariance[14] = odometry.pose.covariance[21] =
      odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = pose_sigma_theta_ * pose_sigma_theta_;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = twist_sigma_linear_ * twist_sigma_linear_;
    odometry.twist.covariance[7] = odometry.twist.covariance[14] =
      odometry.twist.covariance[21] = odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] = twist_sigma_angular_ * twist_sigma_angular_;
    odometry_publisher_->publish(odometry);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = state_.x;
    transform.transform.translation.y = state_.y;
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }

  tf2_ros::TransformBroadcaster transform_broadcaster_;
  CarState state_;
  CarInput input_;
  bool enable_drive_{};
  double wheel_base_{}, wheel_radius_{}, max_steering_{}, servo_angle_{}, command_timeout_{};
  double pose_sigma_x_{}, pose_sigma_y_{}, pose_sigma_theta_{};
  double twist_sigma_linear_{}, twist_sigma_angular_{};
  double speed_{}, yaw_rate_{};
  std::string odom_frame_, base_frame_;
  rclcpp::Time previous_time_, last_command_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr reset_subscription_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostics_timer_;
};
}  // namespace mtrackercar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mtrackercar::Simulator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mtrackercar_sim"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
