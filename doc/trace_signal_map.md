# RAM trace signal map

This document is the authoritative map between signal IDs used by
`TRACE_START`, firmware variables, RAM representation, and names returned by
`MTrackerTrace` in MATLAB.

## Signal IDs

| ID | MATLAB name | Firmware source | Value and unit | RAM type |
|---:|---|---|---|---|
| 1 | `leftWheelVelocity` | `System::drive.wRear` (CAR) or `System::drive.EstimatedLeftVelocity()` (DIFF) | Rear-axle velocity in CAR or estimated left-wheel velocity in DIFF, rad/s | `float32` |
| 2 | `rightWheelVelocity` | `System::drive.EstimatedRightVelocity()` | Estimated right-wheel angular velocity, rad/s | `float32` |
| 3 | `leftWheelPosition` | `System::drive.wheelL.pos` | Left encoder position, encoder counts | `int32` |
| 4 | `rightWheelPosition` | `System::drive.wheelR.pos` | Right encoder position, encoder counts | `int32` |
| 5 | `leftMotorOutput` | `System::drive.regL.u` | Normalized left motor command, limited to `[-1, 1]` | `float32` |
| 6 | `rightMotorOutput` | `System::drive.regR.u` | Normalized right motor command, limited to `[-1, 1]` | `float32` |
| 7 | `odometryX` | `System::odometry.posture.x` | Robot X position, m | `float32` |
| 8 | `odometryY` | `System::odometry.posture.y` | Robot Y position, m | `float32` |
| 9 | `odometryHeading` | `System::odometry.posture.th` | Robot heading, rad | `float32` |
| 10 | `supplyVoltageMv` | INA219 bus voltage if valid; otherwise `System::adcConv.voltageSupply` | Supply voltage, mV | `uint16`, upper word zero |
| 11 | `ina219CurrentDeciMa` | `System::powerMonitor.rawCurrent` | INA219 current in 0.1 mA; MATLAB value divided by 10 gives mA | `int16`, sign-extended to 32 bits |
| 12 | `ina219RawPower` | `System::powerMonitor.rawPower` | INA219 raw power register; one count equals 2 mW with the current calibration | `uint16`, upper word zero |
| 13 | `leftPositionError` | `System::drive.regL.e_phi` | Left controller position error, rad | `float32` |
| 14 | `rightPositionError` | `System::drive.regR.e_phi` | Right controller position error, rad | `float32` |
| 15 | `leftWheelReference` | `System::drive.RearWheelReference()` (CAR) or `System::drive.RefLeft()` (DIFF) | Rear-axle velocity reference in CAR or left-wheel reference in DIFF, rad/s | `float32` |
| 16 | `rightWheelReference` | `System::drive.RefRight()` | Right-wheel angular velocity reference, rad/s | `float32` |

In `DIFF_VEHICLE`, IDs 1 and 2 contain `velocity_reference + observer.z2`; they
are not the unfiltered encoder velocity fields. In `CAR_VEHICLE`, ID 1 contains
`drive.wRear`, calculated from the two independently observed encoder
velocities. ID 15 contains the rear-axle reference used by the CAR controller.
IDs 5 and 6 are controller outputs before conversion to PWM compare values.

The CAR configuration uses one drive controller (`regL`). Therefore IDs 1, 5,
13, and 15 are the relevant motion-control channels. The right-controller
channels 2, 6, 14, and 16 do not represent a second CAR drive controller and
should not be used to assess CAR controller performance.

Wheel signals use the internal controller sign convention. In CAR, positive
IDs 1 and 15 mean physical forward motion. In DIFF, IDs 1 and 15 are stored
without the left-side sign inversion used when command 6 telemetry is encoded.
Consumers comparing a DIFF trace with command 6 must negate those left-wheel
values themselves.

## Record layout in RAM

Every selected signal always occupies two consecutive 16-bit words. Signals
are stored in exactly the order supplied in `TRACE_START`:

```text
sample 0: signalIds[0].low, signalIds[0].high, ..., signalIds[N-1].high
sample 1: signalIds[0].low, signalIds[0].high, ..., signalIds[N-1].high
...
```

The low 16-bit word is stored first. `float32` values contain their raw
IEEE-754 bit pattern. Signed integer values use two's-complement notation.
There is no timestamp in a record; sample time is calculated as:

```text
time_s = sample_index * period_ms / 1000
```

For `N` selected signals, one sample occupies `2 * N` words, or `4 * N`
bytes. The maximum number of samples in the 512 KiB trace buffer is:

```text
maximum_samples = 262144 / (2 * N)
```

## MATLAB interpretation

After `trace = MTrackerTrace('download')`:

- `trace.signalIds(k)` identifies column `k`;
- `trace.signalNames{k}` gives the name from the table above;
- `trace.data(:, k)` contains the converted numeric values;
- `trace.time` contains time in seconds.

Example:

```matlab
MTrackerTrace('start', 2, 5000, [1 2 5 6]);
trace = MTrackerTrace('download');

leftVelocity  = trace.data(:, 1); % signal ID 1, rad/s
rightVelocity = trace.data(:, 2); % signal ID 2, rad/s
leftOutput    = trace.data(:, 3); % signal ID 5, normalized command
rightOutput   = trace.data(:, 4); % signal ID 6, normalized command
```

The order of columns follows the request, not the numerical order of signal
IDs. For example, `[10 1 11]` produces voltage, left velocity, and current in
columns 1, 2, and 3 respectively.

For CAR low-speed controller tuning, a useful 1 ms trace is:

```matlab
MTrackerTrace('start', 1, 10000, [1 5 10 11 13 15]);
trace = MTrackerTrace('download');
```

This records rear-axle velocity, motor output, supply voltage, current,
controller position error, and rear-axle velocity reference for 10 seconds.
