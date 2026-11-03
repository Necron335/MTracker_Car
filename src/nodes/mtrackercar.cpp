#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mtrackercar/communication/protocol.hpp"
#include "mtrackercar/communication/serial_port.hpp"
#include "mtrackercar/control/car_kinematics.hpp"
#include "mtrackercar/msg/car_raw_command.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "mtrackercar/msg/drive_sample.hpp"
#include "mtrackercar/srv/set_regulator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mtrackercar
{
class ResponseTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("mtrackercar"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_rate_(declare_parameter<int>("baud_rate", 921600)),
    robot_address_(declare_parameter<int>("robot_address", 1)),
    enable_drive_(declare_parameter<bool>("enable_drive", true)),
    response_timeout_(declare_parameter<int>("response_timeout_ms", 50)),
    configuration_timeout_(declare_parameter<int>("configuration_timeout_ms", 500)),
    wheel_base_(declare_parameter<double>("wheel_base", 0.190)),
    wheel_radius_(declare_parameter<double>("wheel_radius", 0.033)),
    max_steering_(declare_parameter<double>("max_steering_angle", 0.26)),
    servo_angle_(declare_parameter<double>("servo_angle", 1.0471975511965976)),
    command_timeout_(declare_parameter<double>("command_timeout", 0.5)),
    odometry_topic_(declare_parameter<std::string>("odometry_topic", "/wheel/odom")),
    odom_frame_(declare_parameter<std::string>(
      "odom_frame_id", declare_parameter<std::string>("odom_frame", "odom"))),
    base_frame_(declare_parameter<std::string>(
      "frame_id", declare_parameter<std::string>("base_frame", "base_link"))),
    parser_(checked_robot_address()), transform_broadcaster_(*this),
    pose_sigma_x_(declare_parameter<double>("pose_sigma_x", 0.01)),
    pose_sigma_y_(declare_parameter<double>("pose_sigma_y", 0.01)),
    pose_sigma_theta_(declare_parameter<double>("pose_sigma_theta", 0.02)),
    twist_sigma_linear_(declare_parameter<double>("twist_sigma_linear", 0.02)),
    twist_sigma_angular_(declare_parameter<double>("twist_sigma_angular", 0.03))
  {
    if (!std::isfinite(wheel_base_) || wheel_base_ <= 0 ||
      !std::isfinite(wheel_radius_) || wheel_radius_ <= 0 ||
      !std::isfinite(max_steering_) || max_steering_ <= 0 || max_steering_ >= 1.5707963267948966 ||
      !std::isfinite(servo_angle_) || servo_angle_ < max_steering_ ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0 || response_timeout_.count() <= 0 ||
      configuration_timeout_.count() <= 0 || configuration_timeout_.count() > 2000)
    {
      throw std::invalid_argument("Invalid vehicle geometry, steering limit or timeout");
    }
    const auto legacy_device = declare_parameter<std::string>("serial_port", "");
    if (!legacy_device.empty()) {device_ = legacy_device;}
    const double rate = declare_parameter<double>("command_rate_hz", 100.0);
    if (!std::isfinite(rate) || rate <= 0.0) {
      throw std::invalid_argument("command_rate_hz must be positive");
    }
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    drive_sample_publisher_ = create_publisher<msg::DriveSample>("regulator/drive_sample", 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    command_source_ = declare_parameter<std::string>("command_source", "steering_angle");
    calibration_max_speed_ = declare_parameter<double>("calibration_max_speed", 0.10);
    if (!std::isfinite(calibration_max_speed_) || calibration_max_speed_ <= 0 ||
      calibration_max_speed_ > 0.10)
    {
      throw std::invalid_argument("Calibration speed limit must be in (0, 0.10] m/s");
    }
    if (command_source_ == "steering_angle") {
      drive_subscription_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "drive_command", 1, [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr command) {
          rear_command_ = servo_command_ = 0;
          const auto received = now();
          last_command_time_ = received;
          const double stamp = command->header.stamp.sec + command->header.stamp.nanosec * 1e-9;
          const double age = received.seconds() - stamp;
          if (stamp <= 0 || age < -0.05 || age > command_timeout_ ||
            (!command->header.frame_id.empty() && command->header.frame_id != base_frame_))
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Rejected Ackermann command: stale/invalid timestamp or wrong body frame");
            return;
          }
          if (std::isfinite(command->drive.speed) && std::isfinite(command->drive.steering_angle)) {
            rear_command_ = static_cast<float>(std::clamp(
                  command->drive.speed / wheel_radius_, -127.0, 127.0));
            servo_command_ = static_cast<float>(std::clamp(
                  static_cast<double>(command->drive.steering_angle), -max_steering_, max_steering_));
            // Re-delivery does not extend the lifetime of an old stamped command.
            last_command_time_ = received - rclcpp::Duration::from_seconds(std::max(0.0, age));
          }
        });
    } else if (command_source_ == "cmd_vel") {
      command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          handle_command(*msg);
          });
    } else if (command_source_ == "regulator_tuning") {
      tuning_subscription_ = create_subscription<std_msgs::msg::Float64>(
        "regulator/reference_velocity", 1, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          geometry_msgs::msg::Twist command;
          if (std::isfinite(msg->data) && std::abs(msg->data) <= 0.10) {
            command.linear.x = msg->data;
          }
          handle_command(command);
        });
    } else if (command_source_ == "calibration_raw") {
      raw_subscription_ = create_subscription<msg::CarRawCommand>(
        "calibration/raw_command", 1, [this](msg::CarRawCommand::ConstSharedPtr command) {
          const double speed = command->rear_velocity_rad_s * wheel_radius_;
          rear_command_ = servo_command_ = 0;
          if (std::isfinite(speed) && std::abs(speed) <= calibration_max_speed_ + 1e-9 &&
          std::isfinite(command->raw_servo) && std::abs(command->raw_servo) <= 1.0)
          {
            rear_command_ = static_cast<float>(command->rear_velocity_rad_s);
            servo_command_ = static_cast<float>(command->raw_servo);
          }
          last_command_time_ = now();
        });
    } else {
      throw std::invalid_argument(
              "command_source must be steering_angle, cmd_vel, calibration_raw or regulator_tuning");
    }
    odometry_reset_subscription_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "set_odometry", 1, [this](geometry_msgs::msg::Pose2D::ConstSharedPtr msg) {
        if (std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->theta)) {
          pending_odometry_ = *msg;
        }
      });
    steering_mode_ = declare_parameter<std::string>("steering_mode", "angle");
    if (steering_mode_ != "angle" &&
      !(steering_mode_ == "raw" && command_source_ == "calibration_raw"))
    {
      throw std::invalid_argument("Raw steering is allowed only with command_source=calibration_raw");
    }
    declare_parameter<std::vector<double>>("steering_calibration", {0.0, 1.0, 0.0, 0.0});
    calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "set_steering_calibration", [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        configure(false, *response);
      });
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save_configuration", [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        configure(true, *response);
      });
    regulator_service_ = create_service<srv::SetRegulator>(
      "set_regulator", [this](
        const srv::SetRegulator::Request::SharedPtr request,
        srv::SetRegulator::Response::SharedPtr response) {
        configure_regulator(*request, *response);
      });
    last_command_time_ = now();
    last_reconnect_attempt_ = std::chrono::steady_clock::now() - 2s;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 /
        rate)),
      [this]() {update();});
    diagnostics_timer_ = create_wall_timer(1s, [this]() {publish_diagnostics();});
  }

  ~DriverNode() override
  {
    if (serial_.is_open()) {
      try {
        const auto frame = encode_command(CarCommand{});
        serial_.write_all(frame.data(), frame.size());
      } catch (const std::exception &) {}
    }
  }

private:
  std::vector<std::uint8_t> encode_command(const CarCommand & command)
  {
    auto selected = command;
    selected.steering_mode = command_source_ == "calibration_raw" ?
      SteeringMode::RawServo : SteeringMode::CalibratedAngle;
    return Protocol::encode_car_command(checked_robot_address(), selected, ++sequence_);
  }

  std::uint16_t checked_robot_address() const
  {
    if (robot_address_ < 0 || robot_address_ > 255) {
      throw std::invalid_argument("robot_address must be an 8-bit robot number");
    }
    return static_cast<std::uint16_t>(robot_address_);
  }

  void handle_command(const geometry_msgs::msg::Twist & command)
  {
    const auto input = car_input(command.linear.x, command.angular.z,
      wheel_base_, wheel_radius_, max_steering_, servo_angle_);
    rear_command_ = static_cast<float>(input.rear_velocity);
    servo_command_ = static_cast<float>(input.servo * servo_angle_);
    last_command_time_ = now();
  }

  void disconnect(const std::string & reason)
  {
    if (serial_.is_open()) {
      ++disconnects_;
      RCLCPP_WARN(get_logger(), "Serial connection lost: %s", reason.c_str());
    }
    serial_.close_port();
    awaiting_response_ = false;
    communication_confirmed_ = false;
    previous_telemetry_.reset();
  }

  void reconnect(std::chrono::steady_clock::time_point steady_now)
  {
    if (steady_now - last_reconnect_attempt_ < 1s) {return;}
    last_reconnect_attempt_ = steady_now;
    try {
      serial_.open_port(device_, baud_rate_);
      ++reconnections_;
      RCLCPP_INFO(get_logger(), "Connected to %s", device_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  void update()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!serial_.is_open()) {
      reconnect(steady_now);
      return;
    }
    try {
      std::vector<std::uint8_t> bytes;
      serial_.read_available(bytes);
      for (const auto & frame : parser_.feed(bytes, steady_now)) {
        if (awaiting_response_ && frame.sequence == sequence_) {
          if (const auto telemetry = Protocol::decode_car_telemetry(frame)) {
            awaiting_response_ = false;
            communication_confirmed_ = true;
            publish(*telemetry);
          }
        }
      }
      parser_.expire(steady_now);
      if (awaiting_response_) {
        if (steady_now - request_time_ >= response_timeout_) {
          ++response_timeouts_;
          previous_telemetry_.reset();
          awaiting_response_ = false;
          communication_confirmed_ = false;
        } else {
          return;
        }
      }
      CarCommand command;
      command.steering_mode = command_source_ == "calibration_raw" ?
        SteeringMode::RawServo : SteeringMode::CalibratedAngle;
      const bool can_drive = enable_drive_ && communication_confirmed_ &&
        (now() - last_command_time_).seconds() <= command_timeout_;
      if (pending_odometry_) {
        command.status = 0x0004;
        skip_yaw_samples_ = 2;
        command.x = static_cast<float>(pending_odometry_->x);
        command.y = static_cast<float>(pending_odometry_->y);
        command.theta = static_cast<float>(pending_odometry_->theta);
        pending_odometry_.reset();
      } else if (can_drive) {
        command.status = 0x0003;
        command.rear_velocity = rear_command_;
        command.servo = servo_command_;
      }
      const auto encoded = encode_command(command);
      requested_velocity_ = command.rear_velocity * wheel_radius_;
      serial_.write_all(encoded.data(), encoded.size());
      awaiting_response_ = true;
      request_time_ = steady_now;
    } catch (const std::exception & error) {
      disconnect(error.what());
    }
  }

  Frame wait_response(std::uint8_t command, std::uint8_t sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() + configuration_timeout_;
    std::size_t received_bytes = 0;
    std::size_t unmatched_frames = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      std::vector<std::uint8_t> input;
      serial_.read_available(input);
      received_bytes += input.size();
      for (const auto & frame : parser_.feed(input, std::chrono::steady_clock::now())) {
        if (frame.command == command && frame.sequence == sequence) {return frame;}
        ++unmatched_frames;
      }
      std::this_thread::sleep_for(1ms);
    }
    throw ResponseTimeout("Response timeout: CMD=" + std::to_string(command) +
            " SEQ=" + std::to_string(sequence) + " received_bytes=" +
            std::to_string(received_bytes) + " unmatched_frames=" +
            std::to_string(unmatched_frames) + "; result unknown");
  }

  Frame transact(const std::vector<std::uint8_t> & bytes, std::uint8_t command)
  {
    serial_.write_all(bytes.data(), bytes.size());
    return wait_response(command, sequence_);
  }

  void prepare_configuration()
  {
    rear_command_ = servo_command_ = 0;
    // Complete the outstanding request before sending another serial frame.
    if (awaiting_response_) {
      const auto pending = Protocol::decode_car_telemetry(wait_response(5, sequence_));
      if (!pending) {throw std::runtime_error("Invalid pending motion response");}
      awaiting_response_ = false;
    }
    const auto stopped = Protocol::decode_car_telemetry(
      transact(encode_command(CarCommand{}), 5));
    if (!stopped) {throw std::runtime_error("Invalid stop response");}
    requested_velocity_ = 0;
  }

  void configure_regulator(
    const srv::SetRegulator::Request & request, srv::SetRegulator::Response & response)
  {
    const std::array<float, 4> values{request.feedback_bandwidth, request.observer_bandwidth,
      request.damping, request.inertia};
    // Reject invalid input without touching the robot or its connection.
    try {
      Protocol::encode_regulator(checked_robot_address(), values, 0);
    } catch (const std::exception & error) {
      response.message = error.what();
      return;
    }
    try {
      if (!serial_.is_open() || !communication_confirmed_) {
        throw std::runtime_error("No confirmed serial connection");
      }
      prepare_configuration();
      const auto bytes = Protocol::encode_regulator(checked_robot_address(), values, ++sequence_);
      const auto reply = Protocol::decode_regulator(transact(bytes, 4));
      if (!reply) {throw std::runtime_error("Invalid regulator response; result unknown");}
      response.active_values = reply->coefficients;
      if (reply->status != 0) {
        throw std::runtime_error("Regulator rejected, status=" + std::to_string(reply->status));
      }
      if (reply->coefficients != values) {
        throw std::runtime_error("Firmware returned different active regulator parameters");
      }
      response.success = true;
      response.message = "Regulator active in RAM; save_configuration persists it";
    } catch (const std::exception & error) {
      response.message = error.what();
      disconnect(error.what());
    }
  }

  void configure(bool save, std_srvs::srv::Trigger::Response & response)
  {
    try {
      if (!serial_.is_open() || !communication_confirmed_) {
        throw std::runtime_error("No confirmed serial connection");
      }
      std::array<float, 4> coefficients{};
      if (!save) {
        const auto values = get_parameter("steering_calibration").as_double_array();
        if (values.size() != 4) {
          throw std::invalid_argument("Provide all four calibration coefficients");
        }
        for (std::size_t i = 0; i < 4; ++i) {coefficients[i] = static_cast<float>(values[i]);}
        Protocol::encode_calibration(checked_robot_address(), coefficients, 0);
      }
      // Stop and clear the command before any configuration transaction.
      prepare_configuration();
      if (!save) {
        const auto bytes = Protocol::encode_calibration(checked_robot_address(), coefficients,
            ++sequence_);
        const auto reply = Protocol::decode_calibration(transact(bytes, 13));
        if (!reply || reply->status != 0) {
          throw std::runtime_error("Calibration rejected or invalid response");
        }
        if (reply->coefficients != coefficients) {
          throw std::runtime_error("Firmware returned different active calibration coefficients");
        }
        response.message = "Calibration active in RAM; save_configuration persists it";
      } else {
        bool accepted = false;
        for (const auto expected : {3, 4}) {
          const auto bytes = Protocol::encode_save(checked_robot_address(), false, ++sequence_);
          try {
            const auto status = Protocol::decode_save(transact(bytes, 8));
            if (!status || *status != expected) {
              throw std::runtime_error("EEPROM save sequence rejected at expected status=" +
                      std::to_string(expected));
            }
            accepted = expected == 4;
          } catch (const ResponseTimeout & error) {
            if (expected == 3) {
              throw std::runtime_error(std::string("EEPROM arm not confirmed: ") + error.what());
            }
            // The acceptance reply can be lost while the MCU starts EEPROM I/O.
            // Query the outcome; never resend the save-initiation command.
            RCLCPP_WARN(get_logger(), "EEPROM acceptance unconfirmed; querying status: %s",
              error.what());
          }
        }
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        bool complete = false;
        while (std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(50ms);
          const auto bytes = Protocol::encode_save(checked_robot_address(), true, ++sequence_);
          std::optional<std::uint16_t> status;
          try {
            status = Protocol::decode_save(transact(bytes, 8));
          } catch (const ResponseTimeout & error) {
            RCLCPP_WARN(get_logger(), "EEPROM status temporarily unavailable: %s", error.what());
            continue;
          }
          if (!status) {throw std::runtime_error("Invalid EEPROM status response");}
          if (*status == 0) {complete = true; break;}
          if (*status != 4) {
            throw std::runtime_error("EEPROM save failed, status=" + std::to_string(*status));
          }
          accepted = true;
        }
        if (!complete) {
          throw std::runtime_error(std::string("EEPROM completion timeout; result unknown; ") +
                  "acceptance_confirmed=" + (accepted ? "true" : "false"));
        }
        response.message = "EEPROM save completed";
      }
      response.success = true;
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
      disconnect(error.what());
    }
  }

  void publish(const Telemetry & telemetry)
  {
    const auto stamp = now();
    msg::DriveSample sample;
    sample.stamp_s = stamp.seconds();
    sample.requested_velocity_m_s = requested_velocity_;
    sample.measured_velocity_m_s = wheel_radius_ * telemetry.rear_velocity;
    sample.motor_output = telemetry.control;
    sample.sequence = sequence_;
    drive_sample_publisher_->publish(sample);
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = telemetry.x;
    odometry.pose.pose.position.y = telemetry.y;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, telemetry.theta);
    odometry.pose.pose.orientation = tf2::toMsg(rotation);
    odometry.twist.twist.linear.x = wheel_radius_ * telemetry.rear_velocity;
    // Firmware does not report steering. Derive measured yaw rate from posture.
    if (skip_yaw_samples_ == 0 && previous_telemetry_ &&
      (stamp - previous_stamp_).seconds() > 0.0)
    {
      const double delta = telemetry.theta - previous_telemetry_->theta;
      odometry.twist.twist.angular.z = std::atan2(std::sin(delta), std::cos(delta)) /
        (stamp - previous_stamp_).seconds();
    }
    if (skip_yaw_samples_ > 0) {--skip_yaw_samples_;}
    previous_telemetry_ = telemetry;
    previous_stamp_ = stamp;
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
    transform.transform.translation.x = telemetry.x;
    transform.transform.translation.y = telemetry.y;
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "mtrackercar/serial";
    status.hardware_id = device_;
    status.level = communication_confirmed_ ? status.OK : status.WARN;
    status.message = communication_confirmed_ ? "communicating" : "no confirmed response";
    const auto & counters = parser_.counters();
    const auto add = [&status](const std::string & key, std::uint64_t value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(std::move(item));
      };
    add("valid_frames", counters.valid_frames);
    add("crc_errors", counters.crc_errors);
    add("format_errors", counters.format_errors);
    add("incomplete_frame_timeouts", counters.incomplete_timeouts);
    add("response_timeouts", response_timeouts_);
    add("disconnects", disconnects_);
    add("reconnections", reconnections_);
    if (previous_telemetry_) {
      const auto & t = *previous_telemetry_;
      const auto value = [&status](const std::string & key, const std::string & text) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = text;
          status.values.push_back(item);
        };
      value("voltage_V", std::to_string(t.voltage));
      value("current_A", t.current ? std::to_string(*t.current) : "unavailable");
      value("power_W", t.power ? std::to_string(*t.power) : "unavailable");
      value("power_flags", std::to_string(t.power_flags));
      value("motor_output", std::to_string(t.control));
    }
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tuning_subscription_;
  std::string command_source_, steering_mode_;
  double calibration_max_speed_{};
  rclcpp::Subscription<msg::CarRawCommand>::SharedPtr raw_subscription_;
  std::uint8_t sequence_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_, save_service_;
  std::string device_;
  const int baud_rate_, robot_address_;
  const bool enable_drive_;
  const std::chrono::milliseconds response_timeout_;
  const std::chrono::milliseconds configuration_timeout_;
  const double wheel_base_, wheel_radius_, max_steering_, servo_angle_, command_timeout_;
  const std::string odometry_topic_, odom_frame_, base_frame_;
  SerialPort serial_;
  FrameParser parser_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  const double pose_sigma_x_, pose_sigma_y_, pose_sigma_theta_;
  const double twist_sigma_linear_, twist_sigma_angular_;
  float rear_command_{}, servo_command_{};
  int skip_yaw_samples_{};
  std::optional<Telemetry> previous_telemetry_;
  rclcpp::Time previous_stamp_;
  rclcpp::Time last_command_time_;
  std::optional<geometry_msgs::msg::Pose2D> pending_odometry_;
  bool awaiting_response_{}, communication_confirmed_{};
  std::chrono::steady_clock::time_point request_time_{}, last_reconnect_attempt_{};
  std::uint64_t response_timeouts_{}, disconnects_{}, reconnections_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<msg::DriveSample>::SharedPtr drive_sample_publisher_;
  double requested_velocity_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr odometry_reset_subscription_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostics_timer_;
  rclcpp::Service<srv::SetRegulator>::SharedPtr regulator_service_;
};
}  // namespace mtrackercar

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mtrackercar::DriverNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mtrackercar"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
