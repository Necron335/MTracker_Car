#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "mtrackercar/communication/protocol.hpp"

int main()
{
  using mtrackercar::CarCommand;
  using mtrackercar::Frame;
  using mtrackercar::FrameParser;
  using mtrackercar::Protocol;
  using mtrackercar::SteeringMode;
  CarCommand c;
  const std::array<float, 4> gains{2.0F, 3.0F, 0.5F, 0.01F};
  const auto regulator = Protocol::encode_regulator(1, gains, 9);
  assert(regulator.size() == 24 && regulator[1] == 20 && regulator[3] == 4);
  assert(regulator[5] == 9 && regulator[8] == 0 && regulator[9] == 64);
  Frame regulator_reply{1, 4, 1, {0, 0}, 9};
  regulator_reply.data.insert(regulator_reply.data.end(), regulator.begin() + 6,
    regulator.begin() + 22);
  auto active_regulator = Protocol::decode_regulator(regulator_reply);
  assert(active_regulator && active_regulator->status == 0 &&
    active_regulator->coefficients == gains);
  regulator_reply.command = 13;
  assert(!Protocol::decode_regulator(regulator_reply));
  regulator_reply.command = 4;
  regulator_reply.data.pop_back();
  assert(!Protocol::decode_regulator(regulator_reply));
  for (const float invalid : {0.0F, -1.0F, std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::quiet_NaN()})
  {
    auto invalid_gains = gains;
    invalid_gains[2] = invalid;
    bool failed = false;
    try {
      Protocol::encode_regulator(1, invalid_gains, 0);
    } catch (const std::invalid_argument &) {
      failed = true;
    }
    assert(failed);
  }
  auto bytes = Protocol::encode_car_command(1, c, 42);
  assert(bytes.size() == 24 && bytes[1] == 20 && bytes[3] == 5);
  assert(bytes[4] == 1 && bytes[5] == 42);
  for (int i = 6; i < 22; ++i) {
    assert(bytes[i] == 0);
  }
  c.status = 0xffff; c.rear_velocity = 1; c.servo = 0.25;
  c.x = 1; c.y = -1; c.theta = 1.57079632679F;
  bytes = Protocol::encode_car_command(1, c);
  assert(bytes[6] == 7 && bytes[7] == 0);
  assert(bytes[8] == 0 && bytes[9] == 255);
  assert(bytes[10] == 0 && bytes[11] == 252);
  assert(bytes[12] == 0 && bytes[13] == 128 && bytes[14] == 0);
  assert(bytes[15] == 0 && bytes[16] == 128 && bytes[17] == 255);
  const int heading = bytes[18] | (bytes[19] << 8);
  assert(std::abs(heading / 1024.0 - c.theta) <= 1.0 / 1024);
  assert(bytes[20] == 0 && bytes[21] == 0);
  c.rear_velocity = -1;
  bytes = Protocol::encode_car_command(1, c);
  assert(bytes[8] == 0 && bytes[9] == 1);
  c.rear_velocity = 1;
  c.steering_mode = SteeringMode::CalibratedAngle;
  bytes = Protocol::encode_car_command(1, c);
  assert(bytes[6] == 15 && bytes[10] == 0 && bytes[11] == 4);
  FrameParser parser(1);
  auto now = FrameParser::Clock::now();
  assert(parser.feed(bytes.data(), 7, now).empty());
  assert(parser.feed(bytes.data() + 7, bytes.size() - 7, now).size() == 1);
  bytes[8] ^= 1;
  assert(parser.feed(bytes, now).empty());
  assert(parser.counters().crc_errors == 1);
  assert(parser.feed(Protocol::encode_car_command(2, c), now).empty());
  Frame f{1, 5, 1, {0, 0, 0, 255, 0, 128, 0, 0, 128, 255,
      0, 2, 127, 0x4b, 0x0f, 0x9c, 0x01, 0x0f}, 7};
  auto t = Protocol::decode_car_telemetry(f);
  assert(t && t->rear_velocity == -1 && t->x == 1 && t->y == -1);
  assert(t->theta == 0.5 && t->control == 1);
  assert(std::abs(t->voltage - 12) < 1e-5);
  assert(t->current && std::abs(*t->current + 0.2) < 1e-5);
  assert(t->power && std::abs(*t->power - 0.4) < 1e-5);
  auto parsed = parser.feed(Protocol::encode_frame(f), now);
  assert(parsed.size() == 1 && parsed[0].sequence == 7);
  f.data[17] &= 0xfe;
  t = Protocol::decode_car_telemetry(f);
  assert(t && !t->current && !t->power);
  f.command = 3; assert(!Protocol::decode_car_telemetry(f));
  f.command = 5; f.data.pop_back(); assert(!Protocol::decode_car_telemetry(f));
  c.x = 10000; c.y = -10000; c.rear_velocity = 10000;
  bytes = Protocol::encode_car_command(1, c);
  assert(bytes[8] == 0 && bytes[9] == 128);
  c.rear_velocity = -10000;
  bytes = Protocol::encode_car_command(1, c);
  assert(bytes[8] == 255 && bytes[9] == 127);
  assert(bytes[12] == 255 && bytes[13] == 255 && bytes[14] == 127);
  assert(bytes[15] == 0 && bytes[16] == 0 && bytes[17] == 128);
  c.x = std::numeric_limits<float>::quiet_NaN();
  bool rejected = false;
  try {
    Protocol::encode_car_command(1, c);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  auto calibration = Protocol::encode_calibration(1, {0, 1, -2, 3}, 8);
  assert(calibration.size() == 24 && calibration[1] == 20 && calibration[3] == 13);
  assert(calibration[12] == 128 && calibration[13] == 63);
  Frame reply{1, 13, 1, {0, 0}, 8};
  reply.data.insert(reply.data.end(), calibration.begin() + 6, calibration.begin() + 22);
  auto active = Protocol::decode_calibration(reply);
  assert(active && active->status == 0 && active->coefficients[2] == -2);
  rejected = false;
  try {
    Protocol::encode_calibration(1, {2, 1, 0, 0}, 0);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  auto save = Protocol::encode_save(1, true, 9);
  assert(save[3] == 8 && save[5] == 9 && save[6] == 1 && save.size() == 12);
  assert(Protocol::decode_save(Frame{1, 8, 1, {3, 0}, 9}) == 3);
  assert(!Protocol::decode_save(Frame{1, 5, 1, {3, 0}, 9}));
}
