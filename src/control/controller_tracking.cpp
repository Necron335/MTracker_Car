#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "mtrackercar/control/demo_control.hpp"

namespace mtrackercar
{

class ControllerTracking : public rclcpp::Node
{
public:
    ControllerTracking()
    : Node("mtrackercar_controller_tracking"),
      parameters_(read_parameters()), tracker_(parameters_),
      enabled_(declare_parameter<bool>("enable_motion", false)),
      reference_frame_(declare_parameter<std::string>("reference_frame", "odom")),
      base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
      buffer_(get_clock()), listener_(buffer_, this, false)
    {
        if (reference_frame_.empty() || base_frame_.empty() || reference_frame_ == base_frame_) {
            throw std::invalid_argument("Reference and base frames must be nonempty and different");
        }
        commands_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        const auto qos = rclcpp::QoS(1).transient_local();
        reference_pub_ = create_publisher<nav_msgs::msg::Path>("reference_path", qos);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);
        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("reference_pose", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
        path_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
                if (!trail_.path().poses.empty()) {
                    path_pub_->publish(trail_.path());
                }
            });
        RCLCPP_INFO(get_logger(), "TF feedback: %s <- %s; motion enabled: %s",
            reference_frame_.c_str(), base_frame_.c_str(), enabled_ ? "true" : "false");
    }

    void stop()
    {
        tracker_.stop();
        commands_->publish(geometry_msgs::msg::Twist{});
    }

private:
    FeedbackParameters read_parameters()
    {
        FeedbackParameters p;
        p.radius = declare_parameter<double>("radius", 1.0);
        p.speed = declare_parameter<double>("speed", p.speed);
        p.k_x = declare_parameter<double>("k_x", p.k_x);
        p.k_y = declare_parameter<double>("k_y", p.k_y);
        p.k_theta = declare_parameter<double>("k_theta", p.k_theta);
        p.max_linear = declare_parameter<double>("max_linear", p.max_linear);
        p.max_angular = declare_parameter<double>("max_angular", p.max_angular);
        p.tf_timeout = declare_parameter<double>("tf_timeout", p.tf_timeout);
        return p;
    }

    void tick()
    {
        std::optional<PlanarPose> pose;
        geometry_msgs::msg::TransformStamped transform;
        double stamp = 0.0;
        try {
            transform = buffer_.lookupTransform(reference_frame_, base_frame_, tf2::TimePointZero);
            pose = planar_pose(transform);
            stamp = rclcpp::Time(transform.header.stamp).seconds();
        } catch (const tf2::TransformException &) {
            // No blocking wait: immediately publish zero if feedback is unavailable.
        }
        const double now_seconds = now().seconds();
        const double steady_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto velocity = tracker_.update(pose, stamp, now_seconds, steady_seconds, enabled_);
        geometry_msgs::msg::Twist command;
        command.linear.x = velocity.linear;
        command.angular.z = velocity.angular;
        commands_->publish(command);
        if (was_active_ && !tracker_.active()) {
            RCLCPP_WARN(
                get_logger(), "Feedback unavailable or stale; stopping and pausing reference");
        }
        was_active_ = tracker_.active();
        if (!pose || now_seconds < stamp || now_seconds - stamp > parameters_.tf_timeout) {
            return;
        }
        trail_.append(stamped_pose(*pose, transform.header));
        if (tracker_.anchored()) {
            target_pub_->publish(stamped_pose(
                    tracker_.reference(tracker_.elapsed()), transform.header));
            if (!reference_published_) {
                nav_msgs::msg::Path reference;
                reference.header = transform.header;
                const double period = 2 * std::acos(-1.0) * parameters_.radius / parameters_.speed;
                for (int i = 0; i <= 200; ++i) {
                    reference.poses.push_back(stamped_pose(
                            tracker_.reference(period * i / 200), transform.header));
                }
                reference_pub_->publish(reference);
                reference_published_ = true;
            }
        }
    }

    FeedbackParameters parameters_;
    CircleTracker tracker_;
    bool enabled_;
    std::string reference_frame_;
    std::string base_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    TrajectoryBuffer trail_;
    bool was_active_{false};
    bool reference_published_{false};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;
};

}  // namespace mtrackercar

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<mtrackercar::ControllerTracking>();
        rclcpp::spin(node);
        if (rclcpp::ok()) {
            node->stop();
        }
    } catch (const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("mtrackercar_controller_tracking"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
