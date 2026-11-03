#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "mtrackercar/control/demo_control.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::PoseStamped sample(double x, double y = 0, int seconds = 1)
{
    std_msgs::msg::Header header;
    header.frame_id = "odom";
    header.stamp.sec = seconds;
    return mtrackercar::stamped_pose({x, y, 0}, header);
}

void expect_stopped(const mtrackercar::VelocityCommand & command)
{
    EXPECT_DOUBLE_EQ(command.linear, 0);
    EXPECT_DOUBLE_EQ(command.angular, 0);
}
}  // namespace

TEST(Trajectory, SamplesMeasuredPositionsAndBoundsHistory)
{
    mtrackercar::TrajectoryBuffer trail(3, 0.002);
    trail.append(sample(0));
    trail.append(sample(0.001));
    ASSERT_EQ(trail.path().poses.size(), 1U);
    for (double x : {0.01, 0.02, 0.03}) {
        trail.append(sample(x, 0.1));
    }
    ASSERT_EQ(trail.path().poses.size(), 3U);
    EXPECT_DOUBLE_EQ(trail.path().poses.front().pose.position.x, 0.01);
    EXPECT_DOUBLE_EQ(trail.path().poses.back().pose.position.x, 0.03);
    EXPECT_DOUBLE_EQ(trail.path().poses.back().pose.position.y, 0.1);
    EXPECT_EQ(trail.path().header.frame_id, "odom");
}

TEST(Trajectory, ResetsOnFrameChangeAndTimeRewind)
{
    mtrackercar::TrajectoryBuffer trail;
    trail.append(sample(0, 0, 10));
    auto pose = sample(0.1, 0, 11);
    pose.header.frame_id = "map";
    trail.append(pose);
    ASSERT_EQ(trail.path().poses.size(), 1U);
    EXPECT_EQ(trail.path().header.frame_id, "map");
    pose.header.stamp.sec = 1;
    pose.pose.position.x = 0.2;
    trail.append(pose);
    ASSERT_EQ(trail.path().poses.size(), 1U);
    EXPECT_DOUBLE_EQ(trail.path().poses.front().pose.position.x, 0.2);
    pose.header.frame_id.clear();
    trail.append(pose);
    EXPECT_EQ(trail.path().poses.size(), 1U);
    EXPECT_THROW(mtrackercar::TrajectoryBuffer(1), std::invalid_argument);
    EXPECT_THROW(mtrackercar::TrajectoryBuffer(10, 0), std::invalid_argument);
}

TEST(Feedback, ExtractsAndValidatesTfPosture)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.x = 1;
    transform.transform.translation.y = -2;
    transform.transform.rotation.z = std::sqrt(2.0);
    transform.transform.rotation.w = std::sqrt(2.0);
    const auto pose = mtrackercar::planar_pose(transform);
    ASSERT_TRUE(pose);
    EXPECT_DOUBLE_EQ(pose->x, 1);
    EXPECT_DOUBLE_EQ(pose->y, -2);
    EXPECT_NEAR(pose->theta, kPi / 2, 1e-12);
    transform.transform.rotation.z = 0;
    transform.transform.rotation.w = 0;
    EXPECT_FALSE(mtrackercar::planar_pose(transform));
    transform.transform.translation.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(mtrackercar::planar_pose(transform));
}

TEST(Feedback, CircleStartsTangentAndCloses)
{
    mtrackercar::CircleTracker tracker;
    const auto command = tracker.update(mtrackercar::PlanarPose{1, 2, kPi / 2}, 1, 1, 1);
    EXPECT_DOUBLE_EQ(command.linear, 0.1);
    EXPECT_DOUBLE_EQ(command.angular, 1);
    const auto quarter = tracker.reference(kPi / 2);
    EXPECT_NEAR(quarter.x, 0.9, 1e-12);
    EXPECT_NEAR(quarter.y, 2.1, 1e-12);
    EXPECT_NEAR(quarter.theta, kPi, 1e-12);
    const auto full = tracker.reference(2 * kPi);
    EXPECT_NEAR(full.x, 1, 1e-12);
    EXPECT_NEAR(full.y, 2, 1e-12);
    EXPECT_NEAR(full.theta, kPi / 2, 1e-12);
}

TEST(Feedback, CorrectsPostureAndLimitsVelocities)
{
    mtrackercar::CircleTracker tracker;
    tracker.update(mtrackercar::PlanarPose{}, 1, 1, 1);
    auto command = tracker.update(mtrackercar::PlanarPose{-0.02, -0.01, 0}, 1.02, 1.02, 1.02);
    EXPECT_GT(command.linear, 0.1);
    EXPECT_GT(command.angular, 1);
    command = tracker.update(mtrackercar::PlanarPose{-100, -100, 0}, 1.04, 1.04, 1.04);
    EXPECT_DOUBLE_EQ(command.linear, 0.2);
    EXPECT_DOUBLE_EQ(command.angular, 2);
}

TEST(Feedback, StopsOnMissingDisabledStaleFrozenOrInvalidTf)
{
    mtrackercar::CircleTracker tracker;
    const mtrackercar::PlanarPose pose{};
    expect_stopped(tracker.update(std::nullopt, 0, 1, 1));
    expect_stopped(tracker.update(pose, 1, 1, 1, false));
    EXPECT_FALSE(tracker.anchored());
    tracker.update(pose, 1, 1, 1);
    expect_stopped(tracker.update(pose, 1, 1.3, 1.3));
    EXPECT_DOUBLE_EQ(tracker.elapsed(), 0);
    EXPECT_DOUBLE_EQ(tracker.update(pose, 1.31, 1.31, 1.31).linear, 0.1);
    EXPECT_DOUBLE_EQ(tracker.elapsed(), 0);
    expect_stopped(tracker.update(pose, 1.31, 1.31, 1.6));
    expect_stopped(tracker.update(pose, 3, 2, 2));
    expect_stopped(tracker.update(
            mtrackercar::PlanarPose{std::numeric_limits<double>::quiet_NaN(), 0, 0}, 2, 2, 2));
}

TEST(Feedback, PausesOnClockRewindAndLongTimerGap)
{
    mtrackercar::CircleTracker tracker;
    const mtrackercar::PlanarPose pose{};
    tracker.update(pose, 10, 10, 10);
    expect_stopped(tracker.update(pose, 9, 9, 10.02));
    EXPECT_DOUBLE_EQ(tracker.elapsed(), 0);
    tracker.update(pose, 9.02, 9.02, 10.04);
    expect_stopped(tracker.update(pose, 10, 10, 11));
    EXPECT_DOUBLE_EQ(tracker.elapsed(), 0);
}

TEST(Feedback, RecoversFromDisturbanceInUnicycleSimulation)
{
    mtrackercar::CircleTracker tracker;
    mtrackercar::PlanarPose pose{};
    constexpr double dt = 0.02;
    for (int step = 0; step < 650; ++step) {
        const double now = 1 + step * dt;
        if (step == 50) {
            pose.x += 0.04;
            pose.theta += 0.25;
        }
        const auto command = tracker.update(pose, now, now, now);
        pose.x += command.linear * std::cos(pose.theta) * dt;
        pose.y += command.linear * std::sin(pose.theta) * dt;
        pose.theta += command.angular * dt;
    }
    const auto error = tracker.error();
    EXPECT_LT(std::hypot(error.x, error.y), 0.005);
    EXPECT_LT(std::abs(error.theta), 0.03);
}

TEST(Feedback, RejectsInvalidParameters)
{
    for (double value : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
        mtrackercar::FeedbackParameters parameters;
        parameters.radius = value;
        EXPECT_THROW({mtrackercar::CircleTracker tracker(parameters);}, std::invalid_argument);
    }
    mtrackercar::FeedbackParameters parameters;
    parameters.max_linear = 0.05;
    EXPECT_THROW({mtrackercar::CircleTracker tracker(parameters);}, std::invalid_argument);
    parameters.max_linear = 0.2;
    parameters.max_angular = 0.5;
    EXPECT_THROW({mtrackercar::CircleTracker tracker(parameters);}, std::invalid_argument);
}
