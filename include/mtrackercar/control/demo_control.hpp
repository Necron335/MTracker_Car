#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace mtrackercar
{

struct PlanarPose
{
    double x{};
    double y{};
    double theta{};
};

struct VelocityCommand
{
    double linear{};
    double angular{};
};

inline double wrap_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool finite_pose(const PlanarPose & pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

inline std::optional<PlanarPose> planar_pose(
    const geometry_msgs::msg::TransformStamped & transform)
{
    const auto & p = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    for (const double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
        if (!std::isfinite(value)) {
            return std::nullopt;
        }
    }
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(norm) || norm < 1e-6) {
        return std::nullopt;
    }
    const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
    return PlanarPose{p.x, p.y, std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
}

inline geometry_msgs::msg::PoseStamped stamped_pose(
    const PlanarPose & pose, const std_msgs::msg::Header & header)
{
    geometry_msgs::msg::PoseStamped result;
    result.header = header;
    result.pose.position.x = pose.x;
    result.pose.position.y = pose.y;
    result.pose.orientation.z = std::sin(pose.theta / 2);
    result.pose.orientation.w = std::cos(pose.theta / 2);
    return result;
}

class TrajectoryBuffer
{
public:
    explicit TrajectoryBuffer(int max_points = 10000, double min_distance = 0.002)
    : max_points_(max_points), min_distance_(min_distance)
    {
        if (max_points < 2 || !std::isfinite(min_distance) || min_distance <= 0) {
            throw std::invalid_argument("max_points must be >= 2 and min_distance finite and > 0");
        }
    }

    void append(const geometry_msgs::msg::PoseStamped & pose)
    {
        if (pose.header.frame_id.empty() || !std::isfinite(pose.pose.position.x) ||
            !std::isfinite(pose.pose.position.y))
        {
            return;
        }
        const auto & stamp = pose.header.stamp;
        const auto & last = path_.header.stamp;
        if (pose.header.frame_id != path_.header.frame_id || stamp.sec < last.sec ||
            (stamp.sec == last.sec && stamp.nanosec < last.nanosec))
        {
            path_.poses.clear();
        }
        path_.header = pose.header;
        if (!path_.poses.empty()) {
            const auto & previous = path_.poses.back().pose.position;
            if (std::hypot(pose.pose.position.x - previous.x, pose.pose.position.y - previous.y) <
                min_distance_)
            {
                return;
            }
        }
        path_.poses.push_back(pose);
        if (path_.poses.size() > static_cast<std::size_t>(max_points_)) {
            path_.poses.erase(path_.poses.begin());
        }
    }

    const nav_msgs::msg::Path & path() const {return path_;}

private:
    int max_points_;
    double min_distance_;
    nav_msgs::msg::Path path_;
};

struct FeedbackParameters
{
    double radius{0.1};
    double speed{0.1};
    double k_x{2.0};
    double k_y{40.0};
    double k_theta{10.0};
    double max_linear{0.2};
    double max_angular{2.0};
    double tf_timeout{0.25};
};

class CircleTracker
{
public:
    explicit CircleTracker(const FeedbackParameters & parameters = FeedbackParameters{})
    : parameters_(parameters)
    {
        const auto & p = parameters_;
        for (const double value : {p.radius, p.speed, p.k_x, p.k_y, p.k_theta,
                p.max_linear, p.max_angular, p.tf_timeout})
        {
            if (!std::isfinite(value) || value <= 0) {
                throw std::invalid_argument("Feedback parameters must be finite and positive");
            }
        }
        if (p.speed > p.max_linear || p.speed / p.radius > p.max_angular) {
            throw std::invalid_argument("Reference velocities exceed controller limits");
        }
    }

    PlanarPose reference(double elapsed) const
    {
        const auto & origin = origin_.value();
        const double angle = origin.theta + parameters_.speed / parameters_.radius * elapsed;
        return {
            origin.x + parameters_.radius * (std::sin(angle) - std::sin(origin.theta)),
            origin.y - parameters_.radius * (std::cos(angle) - std::cos(origin.theta)),
            wrap_angle(angle)};
    }

    VelocityCommand stop()
    {
        last_time_.reset();
        active_ = false;
        return {};
    }

    VelocityCommand update(
        const std::optional<PlanarPose> & pose, double stamp, double now,
        double steady_now, bool enabled = true)
    {
        if (!enabled || !pose || !finite_pose(*pose) || !std::isfinite(stamp) ||
            !std::isfinite(now) || !std::isfinite(steady_now) || stamp <= 0 ||
            now - stamp > parameters_.tf_timeout || stamp > now + 0.05)
        {
            return stop();
        }
        if (last_stamp_ && stamp < *last_stamp_) {
            last_stamp_ = stamp;
            stamp_changed_at_ = steady_now;
            return stop();
        }
        if (!last_stamp_ || stamp != *last_stamp_) {
            last_stamp_ = stamp;
            stamp_changed_at_ = steady_now;
        }
        if (steady_now - stamp_changed_at_ > parameters_.tf_timeout) {
            return stop();
        }
        if (!origin_) {
            origin_ = *pose;
        }
        if (last_time_) {
            const double dt = now - *last_time_;
            if (dt < 0 || dt > parameters_.tf_timeout) {
                return stop();
            }
            elapsed_ += dt;
        }
        last_time_ = now;
        const auto target = reference(elapsed_);
        const double dx = target.x - pose->x, dy = target.y - pose->y;
        error_ = {
            std::cos(pose->theta) * dx + std::sin(pose->theta) * dy,
            -std::sin(pose->theta) * dx + std::cos(pose->theta) * dy,
            wrap_angle(target.theta - pose->theta)};
        const auto & p = parameters_;
        const double linear = p.speed * std::cos(error_.theta) + p.k_x * error_.x;
        const double angular = p.speed / p.radius + p.speed *
            (p.k_y * error_.y + p.k_theta * std::sin(error_.theta));
        if (!std::isfinite(linear) || !std::isfinite(angular)) {
            return stop();
        }
        active_ = true;
        return {std::clamp(linear, -p.max_linear, p.max_linear),
            std::clamp(angular, -p.max_angular, p.max_angular)};
    }

    bool anchored() const {return origin_.has_value();}
    bool active() const {return active_;}
    double elapsed() const {return elapsed_;}
    PlanarPose error() const {return error_;}

private:
    FeedbackParameters parameters_;
    std::optional<PlanarPose> origin_;
    std::optional<double> last_time_;
    std::optional<double> last_stamp_;
    double stamp_changed_at_{};
    double elapsed_{};
    PlanarPose error_;
    bool active_{false};
};

}  // namespace mtrackercar
