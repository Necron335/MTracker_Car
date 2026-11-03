#include "mtrackercar/communication/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace mtrackercar
{
namespace
{
void append_u16(std::vector<std::uint8_t> & out, std::uint16_t value)
{
  out.push_back(static_cast<std::uint8_t>(value));
  out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::int16_t read_i16(const std::uint8_t * data)
{
  return static_cast<std::int16_t>(
    static_cast<std::uint16_t>(data[0]) | static_cast<std::uint16_t>(data[1]) << 8);
}

void append_float(std::vector<std::uint8_t> & out, float value)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
    "Protocol requires IEEE-754 binary32");
  if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite command coordinate");}
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<std::uint8_t>(bits >> shift));
  }
}

float read_float(const std::uint8_t * data)
{
  std::uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i) {
    bits |= static_cast<std::uint32_t>(data[i]) << (8 * i);
  }
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::int32_t scaled(float value, float scale, std::int32_t low, std::int32_t high)
{
  if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite wheel velocity");}
  const double result = std::clamp(static_cast<double>(value) * scale,
    static_cast<double>(low), static_cast<double>(high));
  return static_cast<std::int32_t>(std::lround(result));
}
}  // namespace

FrameParser::FrameParser(std::uint16_t robot_address, std::chrono::milliseconds timeout)
: robot_address_(robot_address), timeout_(timeout) {}

void FrameParser::reset()
{
  state_ = State::kMarker;
  content_length_ = 0;
  index_ = 0;
  received_crc_ = 0;
}

void FrameParser::expire(Clock::time_point now)
{
  if (state_ != State::kMarker && now - last_byte_ >= timeout_) {
    ++counters_.incomplete_timeouts;
    reset();
  }
}

std::optional<Frame> FrameParser::feed_byte(std::uint8_t byte, Clock::time_point now)
{
  expire(now);
  if (state_ != State::kMarker || byte == Protocol::kHeader) {last_byte_ = now;}
  switch (state_) {
    case State::kMarker:
      if (byte == Protocol::kHeader) {state_ = State::kLength;}
      break;
    case State::kLength:
      if (byte < 4 || byte > 31) {
        ++counters_.format_errors;
        reset();
        if (byte == Protocol::kHeader) {state_ = State::kLength;}
      } else {
        content_length_ = static_cast<std::size_t>(byte) + 1;
        content_[0] = byte;
        index_ = 1;
        state_ = State::kContent;
      }
      break;
    case State::kContent:
      content_[index_++] = byte;
      if (index_ == content_length_) {state_ = State::kCrcHigh;}
      break;
    case State::kCrcHigh:
      received_crc_ = static_cast<std::uint16_t>(byte) << 8;
      state_ = State::kCrcLow;
      break;
    case State::kCrcLow: {
        received_crc_ |= byte;
        if (received_crc_ != Protocol::crc16(content_.data(), content_length_)) {
          ++counters_.crc_errors;
          reset();
          break;
        }
        const auto address = content_[3];
        if (content_[1] != Protocol::kGroup || address != robot_address_) {
          ++counters_.format_errors;
          reset();
          break;
        }
        Frame frame{content_[1], content_[2], address, {}, content_[4]};
        frame.data.assign(content_.begin() + 5, content_.begin() + content_length_);
        ++counters_.valid_frames;
        reset();
        return frame;
      }
  }
  return std::nullopt;
}

std::vector<Frame> FrameParser::feed(
  const std::uint8_t * data, std::size_t size, Clock::time_point now)
{
  std::vector<Frame> frames;
  for (std::size_t i = 0; i < size; ++i) {
    if (auto frame = feed_byte(data[i], now)) {frames.push_back(std::move(*frame));}
  }
  return frames;
}

std::vector<Frame> FrameParser::feed(const std::vector<std::uint8_t> & data, Clock::time_point now)
{
  return feed(data.data(), data.size(), now);
}

std::uint16_t Protocol::crc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0 ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x8005U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> Protocol::encode_frame(const Frame & frame)
{
  if (frame.data.size() > 27 || frame.robot > 255) {
    return std::vector<std::uint8_t>();
  }
  std::vector<std::uint8_t> out{kHeader, static_cast<std::uint8_t>(4 + frame.data.size()),
    frame.group, frame.command, static_cast<std::uint8_t>(frame.robot),
    frame.sequence};
  out.insert(out.end(), frame.data.begin(), frame.data.end());
  const auto crc = crc16(out.data() + 1, out.size() - 1);
  out.push_back(static_cast<std::uint8_t>(crc >> 8));
  out.push_back(static_cast<std::uint8_t>(crc));
  return out;
}

std::vector<std::uint8_t> Protocol::encode_car_command(
  std::uint16_t robot, const CarCommand & command, std::uint8_t sequence)
{
  Frame frame{kGroup, kCarCommand, robot, {}, sequence};
  const bool angle = command.steering_mode == SteeringMode::CalibratedAngle;
  append_u16(frame.data, (command.status & 7U) | (angle ? 8U : 0U));
  append_u16(frame.data,
      static_cast<std::uint16_t>(scaled(-command.rear_velocity, 256, -32768, 32767)));
  const float steering = angle ? command.servo : -std::clamp(command.servo, -1.0F, 1.0F);
  append_u16(frame.data, static_cast<std::uint16_t>(scaled(steering, 4096, -32768, 32767)));
  for (const auto value : {command.x, command.y}) {
    const auto bits = static_cast<std::uint32_t>(scaled(value, 32768, -8388608, 8388607));
    for (unsigned shift = 0; shift < 24; shift += 8) {
      frame.data.push_back(static_cast<std::uint8_t>(bits >> shift));
    }
  }
  append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.theta, 1024, -32768, 32767)));
  append_u16(frame.data, 0);
  return encode_frame(frame);
}

std::optional<Telemetry> Protocol::decode_car_telemetry(const Frame & frame)
{
  if (frame.group != kGroup || frame.command != 5 || frame.data.size() != 18) {
    return std::nullopt;
  }
  const auto i24 = [](const std::uint8_t * d) {
      const std::int32_t bits = d[0] | (d[1] << 8) | (d[2] << 16);
      return bits >= 0x800000 ? bits - 0x1000000 : bits;
    };
  Telemetry t;
  const auto * d = frame.data.data();
  t.status = static_cast<std::uint16_t>(read_i16(d));
  t.rear_velocity = read_i16(d + 2) / 256.0F;
  t.x = i24(d + 4) / 32768.0F;
  t.y = i24(d + 7) / 32768.0F;
  t.theta = read_i16(d + 10) / 1024.0F;
  t.control = (d[12] < 128 ? static_cast<int>(d[12]) : static_cast<int>(d[12]) - 256) / 127.0F;
  t.voltage = ((d[13] << 4) | (d[14] >> 4)) * 0.010F;
  t.power_flags = d[17] & 15;
  if (t.power_flags & 1) {
    int current = ((d[14] & 15) << 8) | d[15];
    if (current & 0x800) {current -= 4096;}
    t.current = current * 0.002F;
    t.power = ((d[16] << 4) | (d[17] >> 4)) * 0.025F;
  }
  return t;
}

std::vector<std::uint8_t> Protocol::encode_calibration(
  std::uint16_t robot, const std::array<float, 4> & coefficients, std::uint8_t sequence)
{
  Frame frame{kGroup, 13, robot, {}, sequence};
  for (std::size_t i = 0; i < coefficients.size(); ++i) {
    const float limit = i == 0 ? 1.0F : 1000.0F;
    if (!std::isfinite(coefficients[i]) || std::abs(coefficients[i]) > limit) {
      throw std::invalid_argument("Invalid steering calibration");
    }
    append_float(frame.data, coefficients[i]);
  }
  return encode_frame(frame);
}

std::optional<CalibrationReply> Protocol::decode_calibration(const Frame & frame)
{
  if (frame.group != kGroup || frame.command != 13 || frame.data.size() != 18) {
    return std::nullopt;
  }
  CalibrationReply reply;
  reply.status = static_cast<std::uint16_t>(read_i16(frame.data.data()));
  for (std::size_t i = 0; i < 4; ++i) {
    reply.coefficients[i] = read_float(frame.data.data() + 2 + 4 * i);
    if (!std::isfinite(reply.coefficients[i]) ||
      std::abs(reply.coefficients[i]) > (i == 0 ? 1.0F : 1000.0F)) {return std::nullopt;}
  }
  return reply;
}

std::vector<std::uint8_t> Protocol::encode_regulator(
  std::uint16_t robot, const std::array<float, 4> & values, std::uint8_t sequence)
{
  if (values[0] < 1.0F || values[0] > 200.0F || values[1] < values[0] ||
    values[1] > 1000.0F || values[2] < 0.1F || values[2] > 5.0F ||
    values[3] < 0.00001F || values[3] > 1.0F)
  {
    throw std::invalid_argument("Regulator parameters outside firmware ranges");
  }
  Frame frame{kGroup, 4, robot, {}, sequence};
  for (const float value : values) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("Regulator parameters must be finite and positive");
    }
    append_float(frame.data, value);
  }
  return encode_frame(frame);
}

std::optional<CalibrationReply> Protocol::decode_regulator(const Frame & frame)
{
  if (frame.group != kGroup || frame.command != 4 || frame.data.size() != 18) {
    return std::nullopt;
  }
  CalibrationReply reply;
  reply.status = static_cast<std::uint16_t>(read_i16(frame.data.data()));
  for (std::size_t i = 0; i < 4; ++i) {
    reply.coefficients[i] = read_float(frame.data.data() + 2 + 4 * i);
    if (!std::isfinite(reply.coefficients[i])) {return std::nullopt;}
  }
  return reply;
}

std::vector<std::uint8_t> Protocol::encode_save(
  std::uint16_t robot, bool query, std::uint8_t sequence)
{
  return encode_frame(Frame{kGroup, 8, robot, {static_cast<std::uint8_t>(query), 0, 0, 0},
             sequence});
}

std::optional<std::uint16_t> Protocol::decode_save(const Frame & frame)
{
  if (frame.group != kGroup || frame.command != 8 || frame.data.size() != 2) {return std::nullopt;}
  return static_cast<std::uint16_t>(read_i16(frame.data.data()));
}

}  // namespace mtrackercar
