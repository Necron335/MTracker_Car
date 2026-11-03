# MTracker CAR driver migration guide

This document is an implementation handoff for an agent updating the PC/MATLAB
robot driver to the current `CAR_VEHICLE` firmware protocol. Treat the byte
layouts below as authoritative. The old `CMD=5` layout containing three
`float32` odometry fields is no longer accepted.

## Scope and required changes

The driver must:

1. encode CAR motion with fixed-point `CMD=5`, `LEN=20`;
2. decode the fixed-point `CMD=5`, `LEN=22` response;
3. accept the response as `CMD=5` (old firmware incorrectly returned `CMD=3`);
4. support raw-servo and calibrated-angle steering modes;
5. support steering calibration through `CMD=13`;
6. preserve `ROBOT` and `SEQ` validation and the existing CRC behavior.

Do not change the DIFF path. `DIFF_VEHICLE` continues to use `CMD=6`.

## Common serial frame

All multibyte values are little-endian. Signed integers use two's complement.

```text
+------+-----+-------+-----+-------+-----+------+-------+-------+
| 0xAA | LEN | GROUP | CMD | ROBOT | SEQ | DATA | CRC_H | CRC_L |
+------+-----+-------+-----+-------+-----+------+-------+-------+
```

- `GROUP` is `0x01`.
- `LEN` counts bytes from `GROUP` through the end of `DATA`.
- Total serial frame size is `LEN + 4` bytes.
- CRC-16 polynomial: `0x8005`.
- CRC initial value: `0xFFFF`.
- No final XOR.
- CRC input starts at `LEN` and contains the `LEN` byte plus the following
  `LEN` bytes. CRC is appended high byte first, then low byte.
- USB/SCI-C settings: 921600 bit/s, 8 data bits, no parity, 1 stop bit.

For nRF24, omit `0xAA` and serial CRC. The radio payload begins with `LEN`, is
32 bytes long, and only the first `LEN + 1` bytes are meaningful. Match radio
responses using both `CMD` and `SEQ`.

## CMD=5 request: CAR motion and optional odometry update

Normal ROS input defaults to `drive_command` (`ackermann_msgs/msg/AckermannDriveStamped`):
`drive.speed` and physical `drive.steering_angle`. Firmware evaluates its calibration
polynomial; the ROS driver does not apply that polynomial a second time.
All normal and regulator-tuning commands use status bit 3 (calibrated angle).
Only `command_source=calibration_raw` accepts normalized raw servo input.
`steering_mode=raw` with any other source is rejected at startup.

The request uses `LEN=20`, so `DATA` is exactly 16 bytes.

| DATA offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 2 | `STATUS` | unsigned 16-bit mask |
| 2 | 2 | rear-wheel velocity | signed Q8, see sign rule below |
| 4 | 2 | steering command | signed Q12 |
| 6 | 3 | odometry X | signed Q15, metres |
| 9 | 3 | odometry Y | signed Q15, metres |
| 12 | 2 | odometry heading | signed Q10, radians |
| 14 | 2 | reserved | write zero |

### STATUS bits

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x0001` | enable the drive controller and apply velocity/steering |
| 1 | `0x0002` | enable the motor power stage |
| 2 | `0x0004` | replace robot odometry with X/Y/heading from this request |
| 3 | `0x0008` | interpret steering Q12 as an angle in radians |
| 15..4 | — | reserved; write zero |

### Request conversions

Use saturation before converting to an integer.

```text
wireVelocityQ8 = round(-requestedRearVelocityRadPerSec * 256)
wireXQ15       = round(xMetres * 32768)       // signed 24-bit
wireYQ15       = round(yMetres * 32768)       // signed 24-bit
wireHeadingQ10 = round(headingRadians * 1024) // signed 16-bit
```

The minus sign on rear-wheel velocity is required by the current firmware
coordinate convention. Do not remove it during this migration.

For steering, select exactly one API mode:

#### Raw-servo mode

- Clear `STATUS & 0x0008`.
- Public API range: `rawServo` in `[-1, 1]`.
- Current firmware preserves a legacy sign inversion:

```text
wireSteeringQ12 = round(-rawServo * 4096)
```

Thus a driver-level `rawServo=+0.25` must send `-1024`. The firmware clamps the
resulting internal servo command to `[-1, 1]`.

#### Calibrated-angle mode

- Set `STATUS |= 0x0008`.
- Public API value is the physical steering angle in radians.

```text
wireSteeringQ12 = round(angleRadians * 4096)
```

Firmware applies:

```text
rawServo = clamp(c0 + c1*a + c2*a^2 + c3*a^3, -1, +1)
```

where `a` is `angleRadians`. Evaluate the polynomial with Horner's method:
`rawServo = ((c3*a + c2)*a + c1)*a + c0`. The coefficient signs define the
physical steering direction. Do not apply the raw-mode sign inversion in angle
mode.

### Signed 24-bit helper

Clamp to `[-8388608, 8388607]`, cast to a signed integer, and emit the least
significant three bytes. To decode, assemble three bytes and sign-extend bit 23.

## CMD=5 response

The response has `CMD=5`, `LEN=22`, and exactly 18 bytes of `DATA`.

| DATA offset | Size | Field | Decoding |
|---:|---:|---|---|
| 0 | 2 | status | unsigned 16-bit; currently zero |
| 2 | 2 | rear-wheel velocity | signed value / 256, rad/s |
| 4 | 3 | odometry X | signed value / 32768, m |
| 7 | 3 | odometry Y | signed value / 32768, m |
| 10 | 2 | odometry heading | signed value / 1024, rad |
| 12 | 1 | motor output | signed int8 / 127, nominally `[-1,1]` |
| 13 | 5 | packed power telemetry | described below |

The firmware reports the internal rear-wheel measurement with its physical
sign: positive means forward motion. The driver divides the received signed Q8
value by 256 and does not negate it.

### Packed power telemetry

Let the five bytes at offsets 13..17 be `b0..b4`:

```text
voltageRaw = (b0 << 4) | (b1 >> 4)
currentRaw = ((b1 & 0x0F) << 8) | b2
powerRaw   = (b3 << 4) | (b4 >> 4)
flags      = b4 & 0x0F
```

Sign-extend `currentRaw` as a signed 12-bit number.

```text
voltageV = voltageRaw * 0.010
currentA = signedCurrentRaw * 0.002
powerW   = powerRaw * 0.025
```

Flags:

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x01` | current and power values are valid |
| 1 | `0x02` | voltage came from INA219; otherwise it came from ADC |
| 2 | `0x04` | INA219 was configured successfully |
| 3 | `0x08` | power-monitor support is enabled in firmware |

If bit 0 is clear, expose current and power as unavailable rather than treating
their zero payloads as confirmed physical zero.

## CMD=13: steering calibration

This configuration command intentionally still uses IEEE-754 `float32`.

Request: `CMD=13`, `LEN=20`, 16 bytes of `DATA`:

| DATA offset | Size | Field | Valid range |
|---:|---:|---|---:|
| 0 | 4 | `c0` | `-1..1` |
| 4 | 4 | `c1` | `-1000..1000` |
| 8 | 4 | `c2` | `-1000..1000` |
| 12 | 4 | `c3` | `-1000..1000` |

Response: `CMD=13`, `LEN=22`, 18 bytes of `DATA`:

| DATA offset | Size | Field |
|---:|---:|---|
| 0 | 2 | configuration status |
| 2 | 4 | active `c0` |
| 6 | 4 | active `c1` |
| 10 | 4 | active `c2` |
| 14 | 4 | active `c3` |

Changing calibration updates RAM immediately but does not by itself write the
EEPROM.

The external driver must expose a method that sends all four coefficients in
one transaction. Do not send partial coefficient updates and do not encode the
coefficients as Q12; Q12 applies only to the steering command in `CMD=5`.

## CMD=8: persist configuration in EEPROM

The ROS driver never blindly retries either save-initiation request. After
confirmed arming it sends acceptance once; a missing acceptance reply is
resolved with read-only status queries. Individual query timeouts are tolerated
within the 5-second completion deadline. Only status zero confirms completion;
a missing arm reply or exhausted completion deadline remains an unknown outcome.

The M24C64 EEPROM record contains geometry, controller parameters, and steering
calibration. A save requires two consecutive accepted `CMD=8` requests:

1. first response status: `CONFIG_SAVE_ARMED` (`3`);
2. second response status: `CONFIG_SAVE_ACCEPTED` (`4`);
3. firmware writes and verifies EEPROM after the accepted response is drained;
4. query save completion by sending `CMD=8` with the first DATA byte equal to
   `1`. Pad DATA to four bytes if required by the existing driver framing code.

Final save status is `CONFIG_OK` (`0`) or `CONFIG_STORAGE_ERROR` (`2`). Any other
command between the two save requests cancels the armed sequence.

## Other persistent configuration commands

These commands are unchanged, but their values are now persisted in the same
M24C64 record when `CMD=8` completes:

- `CMD=4`, `LEN=20`: four little-endian `float32` values in this order:
  `feedbackBandwidth`, `observerBandwidth`, `damping`, `inertia`. Response has
  `LEN=22`: status followed by the four active values.
- `CMD=7`, `LEN=16`: three little-endian `float32` values in millimetres:
  `leftWheelRadiusMm`, `rightWheelRadiusMm`, `wheelBaseMm`. Response has
  `LEN=18`: status followed by the three active values.

Configuration status values are: `0` OK, `1` invalid data, `2` storage error,
`3` save armed, `4` save accepted, `5` save not requested, and `6` save
cancelled.

The verified `CAR_VEHICLE` defaults are:

```text
feedbackBandwidth = 15.0 rad/s
observerBandwidth = 80.0 rad/s
damping            = 1.0
inertia            = 0.0025
```

`observerBandwidth` configures the motor-controller ESO. A separate encoder
position-to-velocity observer runs at 1 kHz with a fixed bandwidth of 20 rad/s;
it is not part of `CMD=4`. The firmware currently has no motor dead-zone or
static-friction compensation parameter.

## Response validation

Reject a serial response unless all conditions hold:

1. start byte is `0xAA`;
2. `GROUP == 1`;
3. exactly `LEN + 4` bytes were received;
4. CRC is valid;
5. `ROBOT` matches the selected robot;
6. `CMD` matches the request (`5` for CAR motion);
7. for radio, `SEQ` also matches the pending transaction.

Do not retain compatibility code that expects a CAR response with `CMD=3`.

## Suggested driver API

Keep raw calibration and normal operation explicit rather than overloading one
untyped numeric argument:

```text
sendCarRaw(rearVelocityRadPerSec, rawServo, status, optionalPose)
sendCarAngle(rearVelocityRadPerSec, steeringAngleRad, status, optionalPose)
setSteeringCalibration(c0, c1, c2, c3)
saveConfiguration()
```

Internally both CAR motion methods create `CMD=5`; they differ only in status
bit 3 and steering conversion.

## Minimum acceptance tests

1. Encoding zero motion produces `LEN=20`, `CMD=5`, and 16 DATA bytes.
2. Raw `+0.25` produces steering bytes for signed Q12 value `-1024`.
3. Angle `+0.25 rad` produces steering bytes for signed Q12 value `+1024` and
   sets status bit 3.
4. X=`1.0 m` encodes as signed 24-bit `32768`; X=`-1.0 m` sign-extends correctly.
5. Heading=`pi/2` round-trips within one Q10 count.
6. A response with `CMD=3` is rejected for a pending `CMD=5` transaction.
7. A valid `CMD=5`, `LEN=22` response decodes all fields at the exact offsets
   listed above.
8. Signed 12-bit negative current decodes correctly.
9. Invalid CRC, wrong robot number, or wrong radio sequence is rejected.
10. Calibration is applied immediately, but survives reset only after the
    two-step `CMD=8` save completes successfully.

## Firmware references

- `src/protocol/commands_interpreter.cpp`: `CMD=5`, `CMD=8`, and `CMD=13`
- `src/protocol/telemetry_encoder.cpp`: fixed-point response encoding
- `src/protocol/motion_command.cpp`: analogous DIFF `CMD=6` implementation
- `headers/protocol/frame_codec.h`: byte helpers
- `doc/communication_protocol.md`: complete protocol description

## ROS regulator service

The driver also publishes `regulator/drive_sample` (`mtrackercar/msg/DriveSample`)
for each matched periodic CMD=5 reply. It contains host receipt time, the velocity
in the corresponding request, measured velocity and motor output. The request
echo is not the internal firmware reference. This enables explicit
`cmd5_telemetry` tuning without an undocumented TRACE adapter; it does not
provide RAM samples or internal controller position error.

The driver exposes CMD=4 through `/set_regulator` (`mtrackercar/srv/SetRegulator`).
USB/SCI returns its reply directly; no separate read or nRF24 POLL is needed.
No response-time guarantee is specified. The driver drains the outstanding
motion transaction before sending stop/configuration and uses
`configuration_timeout_ms` (default 500 ms) independently of the periodic
motion timeout. A timeout reports expected CMD/SEQ and received byte/frame counts.
All four finite, positive values are sent together, following a disabled-drive
command. The status and all echoed float32 values must match. A successful call
changes RAM; `/save_configuration` persists the whole configuration separately.
The provided guide does not define buffer capture/read commands or their sample
layout. CMD=5 does not expose the controller error or a high-rate sample buffer.
