#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mtrackercar
{

struct Frame
{
  std::uint8_t group{};
  std::uint8_t command{};
  std::uint16_t robot{};
  std::vector<std::uint8_t> data;
  std::uint8_t sequence{};
};

struct ParserCounters
{
  std::uint64_t valid_frames{};
  std::uint64_t crc_errors{};
  std::uint64_t format_errors{};
  std::uint64_t incomplete_timeouts{};
};

class FrameParser
{
public:
  using Clock = std::chrono::steady_clock;
  explicit FrameParser(
    std::uint16_t robot_address, std::chrono::milliseconds timeout =
    std::chrono::milliseconds(10));
  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size, Clock::time_point now);
  std::vector<Frame> feed(const std::vector<std::uint8_t> & data, Clock::time_point now);
  void expire(Clock::time_point now);
  const ParserCounters & counters() const {return counters_;}

private:
  enum class State {kMarker, kLength, kContent, kCrcHigh, kCrcLow};
  void reset();
  std::optional<Frame> feed_byte(std::uint8_t byte, Clock::time_point now);
  State state_{State::kMarker};
  std::array<std::uint8_t, 32> content_{};
  std::size_t content_length_{};
  std::size_t index_{};
  std::uint16_t received_crc_{};
  std::uint16_t robot_address_;
  std::chrono::milliseconds timeout_;
  Clock::time_point last_byte_{};
  ParserCounters counters_{};
};

enum class SteeringMode {RawServo, CalibratedAngle};

struct CarCommand
{
  std::uint16_t status{};
  float rear_velocity{};  // rad/s, positive forward
  SteeringMode steering_mode{SteeringMode::RawServo};
  float servo{};  // RawServo: [-1,1]; CalibratedAngle: physical radians
  float x{}, y{}, theta{};
};

struct Telemetry
{
  std::uint16_t status{};
  float voltage{};
  std::optional<float> current, power;
  std::uint8_t power_flags{};
  float rear_velocity{};
  float x{}, y{}, theta{};
  float control{};
};

struct CalibrationReply
{
  std::uint16_t status{};
  std::array<float, 4> coefficients{};
};

class Protocol
{
public:
  static constexpr std::uint8_t kHeader = 0xAA;
  static constexpr std::uint8_t kGroup = 1;
  static constexpr std::uint8_t kCarCommand = 5;
  static std::uint16_t crc16(const std::uint8_t * data, std::size_t size);
  static std::vector<std::uint8_t> encode_frame(const Frame & frame);
  static std::vector<std::uint8_t> encode_car_command(
    std::uint16_t robot, const CarCommand & command, std::uint8_t sequence = 0);
  static std::vector<std::uint8_t> encode_calibration(
    std::uint16_t robot, const std::array<float, 4> & coefficients, std::uint8_t sequence);
  static std::optional<CalibrationReply> decode_calibration(const Frame & frame);
  static std::vector<std::uint8_t> encode_regulator(
    std::uint16_t robot, const std::array<float, 4> & values, std::uint8_t sequence);
  static std::optional<CalibrationReply> decode_regulator(const Frame & frame);
  static std::vector<std::uint8_t> encode_save(
    std::uint16_t robot, bool query, std::uint8_t sequence);
  static std::optional<std::uint16_t> decode_save(const Frame & frame);
  static std::optional<Telemetry> decode_car_telemetry(const Frame & frame);
};

}  // namespace mtrackercar
