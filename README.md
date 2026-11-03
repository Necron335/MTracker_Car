# MTracker Car — ROS 2

ROS 2 package `mtrackercar` for MTrackerFirmware built with `CAR_VEHICLE`.
Uses the structure of MTracker-ROS: a serial/protocol library, driver,
simple velocity controller, TF circle tracking, trajectory visualization,
odometry reset demo, configuration and launch files. The other repositories
are reference sources and are not modified.

## Build and start

PC undervoltage monitoring is described in the
[Polish NUC power-monitor manual](doc/pc_power_monitor_pl.md).
It ships unconfigured and with automatic shutdown disabled.

Regulator tuning via `/set_regulator` (`CMD=4`) is described in
[the Polish regulator manual](doc/regulator_manual_pl.md)
([printable HTML](doc/regulator_manual_pl.html)).

On Linux with a ROS 2 distribution and dependencies from `package.xml` installed.
Replace `<distro>` below with your installed ROS 2 distribution:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-up-to mtrackercar --symlink-install
source install/setup.bash
ros2 launch mtrackercar mtrackercar_master.launch.py
```

Edit `config/mtrackercar.yaml` for the serial device and vehicle geometry.
The serial implementation uses Linux termios. Defaults: 921600 baud,
8-bit robot number 1 (set it to match the firmware), wheel radius 0.033 m.
`wheel_base=0.190` comes from the car firmware, but is used here as an effective
bicycle wheelbase: measure and calibrate it for steering conversion. Firmware
odometry geometry is independent and is never changed by this driver.

First communication check with drive disabled:

```bash
ros2 run mtrackercar mtrackercar_drv --ros-args \
  --params-file /path/to/config/mtrackercar.yaml -p enable_drive:=false
ros2 topic echo /diagnostics
```

## Motion and demos

For a repeated forward/reverse steering demonstration within a 0.5 m workspace
radius, use `compact_demo.launch.py enable_motion:=true` and call
`/compact_demo/start`. Target speed is 0.25 m/s; RViz shows the measured trail,
robot orientation and commanded steering. See [the compact demo manual](doc/compact_demo_pl.md)
([HTML version](doc/compact_demo_pl.html)).

```bash
ros2 launch mtrackercar compact_demo.launch.py enable_motion:=true
# In another terminal with the workspace environment loaded:
ros2 service call /compact_demo/start std_srvs/srv/Trigger '{}'
# Cancel:
ros2 service call /compact_demo/cancel std_srvs/srv/Trigger '{}'
```

This finite sequence has eight forward/reverse pairs (16 passes of about 30 cm),
lasting roughly 30–35 s. Steering changes halfway through each pass without
stopping; stops occur only before reversing and at completion. Reverse steering
order mirrors the forward pass to retrace the path in the kinematic model.
The planned total path is about 4.8 m. Odometry triggers a stop at 0.40 m radius,
6 m total travel or 60 s, leaving margin to the requested 0.5 m displacement.
The 0.5 m limit is not the accumulated path length. These are rear-axle odometry limits;
they cannot guarantee the physical vehicle footprint or braking distance.
The launch opens RViz but motion additionally requires the start service.
The RViz view uses a 5 cm grid, measured trajectory and orientation, a schematic
body, front wheels showing commanded steering, and a 0.4 m radius boundary.

Normal mode defaults to `command_source=steering_angle` and subscribes to
`drive_command` (`ackermann_msgs/msg/AckermannDriveStamped`): `drive.speed` in m/s and
`drive.steering_angle` as the physical virtual-front-wheel angle in radians. The driver clamps
the angle to `max_steering_angle` and sends it to firmware in calibrated-angle
mode (CMD=5 status bit 3). Firmware applies its stored calibration polynomial.
The angle is neither normalized by `servo_angle` nor negated for reverse motion.
Nonfinite commands stop motion; the command timeout still applies.
The header must have a nonzero current timestamp, no older than `command_timeout`
and at most 50 ms into the future. `frame_id` must be empty or match `base_frame`.
Expiry uses the command timestamp, so repeated delivery cannot keep an old command alive.
Only speed and steering angle are implemented; acceleration, jerk and steering-rate
fields are not enforced. Leave those fields zero. The custom `CarDriveCommand`
input has been replaced by the standard message type.

```bash
ros2 topic pub --rate 20 /drive_command ackermann_msgs/msg/AckermannDriveStamped \
  '{header: auto, drive: {speed: 0.05, steering_angle: 0.20}}'
```

`header: auto` refreshes the timestamp on each publication in ROS 2 Jazzy.
The example starts motion. Stop the publisher to invoke the command watchdog.
The measured calibration covered `[-0.2630, +0.3289] rad` (about -15.1 to +18.8 deg).
The default symmetric limit is `0.26 rad` (14.9 deg), within both measured sides;
this is not a claim about the mechanism's absolute end stops.

Build the dependency as well: `colcon build --packages-up-to mtrackercar --symlink-install`.
This workspace includes the official ROS 2 `ackermann_msgs` source in `src/ackermann_msgs`.

For existing Twist publishers, launch with `command_source:=cmd_vel`.
`cmd_vel` is `geometry_msgs/msg/Twist`: `linear.x` in m/s and `angular.z` in
rad/s yaw rate. This changes the old car driver's interpretation of angular.z
as a steering angle. The driver converts using
`steering=atan(wheel_base*yaw_rate/speed)`, clamps steering to
`max_steering_angle` (default 0.26 rad), and sends the physical angle to firmware (`steering_mode: angle`, default).
Raw servo commands are accepted only with `command_source=calibration_raw`
on `calibration/raw_command`; `steering_mode: raw` is rejected in normal modes.
In the Twist compatibility mode, reverse motion changes the
steering sign correctly. Zero speed produces zero drive/steering; a car cannot
rotate in place. Nonfinite commands stop motion. Steering saturation means
the requested yaw rate may be unattainable. The steering calibration and direction require hardware verification.

```bash
ros2 launch mtrackercar circle_demo.launch.py enable_motion:=true
ros2 launch mtrackercar feedback_circle.launch.py enable_motion:=true
```

Run one demo at a time. Both default to radius 1.00 m and speed 0.10 m/s,
with motion disabled unless explicitly enabled. `start_driver:=false` reuses
an existing driver; `rviz:=false` disables RViz. Feedback uses fresh TF
`odom -> base_link` and stops/pauses when TF is invalid or stale. It is the
reference package's unicycle tracking law followed by the car steering limit;
it is not a car-specific path planner and cannot guarantee recovery when the
required curvature exceeds the vehicle's capabilities.

`controller_simple.launch.py` runs only the constant-velocity controller.
Stop other `cmd_vel` publishers before using a demo; there is no command arbiter.
Missing commands stop the driver after `command_timeout` (default 0.5 s).
Communication must be confirmed before drive commands are sent. Reconnection,
CRC errors, response timeouts and parser errors are reported in `/diagnostics`.

## Kinematic simulator

`mtrackercar_sim` replaces the hardware driver with an ideal rear-axle bicycle
model. It uses the same `cmd_vel` (m/s and yaw rad/s), `set_odometry` (Pose2D),
`/wheel/odom`, `odom -> base_link` TF and `/diagnostics` topic types. Do not run
the simulator and hardware driver together on these same topics/frames.

```bash
ros2 launch mtrackercar mtrackercar_sim.launch.py rviz:=true
```

Then, in a second terminal, run one controller, for example:

```bash
ros2 launch mtrackercar mtrackercar_joy.launch.py start_driver:=false enable_motion:=true
# Alternatively, run one circle demo:
ros2 launch mtrackercar feedback_circle.launch.py start_driver:=false rviz:=false enable_motion:=true
```

It can also run directly with defaults:

```bash
ros2 run mtrackercar mtrackercar_sim
```

The simulator reuses `config/mtrackercar.yaml`; the node name remains
`mtrackercar` so the same parameter file applies. Geometry, steering limits,
`enable_drive`, `command_timeout`, `command_rate_hz`, frame/topic names and
covariance parameters match the driver. Serial settings are ignored. The
optional RViz trajectory assumes the default `/wheel/odom` topic and `odom`
frame. Match joystick geometry to the simulator when overriding defaults.

Motion obeys `v=rear_wheel_speed*wheel_radius` and
`yaw_rate=v*tan(steering)/wheel_base`, using the driver's input conversion.
The model integrates constant-curvature arcs exactly, including reverse motion
and steering saturation. Stale commands stop at the command timeout; invalid
commands stop immediately on receipt. Resetting odometry sets posture and
clears the active command. Odometry reports exact model velocities rather than
estimating yaw rate from serial samples. Diagnostics identify simulation instead
of reporting serial counters. Covariances are published metadata, not injected
noise. There is no motor lag, tire slip, collision model or serial emulation.

Integration and command age use ROS time, with wall timers scheduling updates.
The simulator does not publish `/clock`; normal wall time works by default. If
`use_sim_time` is enabled, provide an external `/clock`. A backward clock jump
clears the active command. Numerical model tests run without ROS; ROS topic/TF
integration remains unverified in the current environment.

## Joystick control

The `mtrackercar_joy` node adapts the supplied joystick controller to the driver's
yaw-rate interface. Launch the driver, `joy_node` and joystick controller together:

```bash
ros2 launch mtrackercar mtrackercar_joy.launch.py enable_motion:=true
```

Motion is disabled by default. Axis 1 controls speed and axis 0 steering, with
the original clamp behavior. Button indices 0/1/2/3 select manual/stop/constant
left circle/constant right circle. Modes remain selected after button release;
button 1 selects stop. Button labels depend on the joystick mapping. Defaults
are 0.5 m/s maximum speed and 0.6 rad maximum steering.

Launch arguments include `device_id`, `max_speed`, `max_steering_angle`,
`wheel_base`, `config`, `start_driver` and `start_joy`. Use
`start_driver:=false` or `start_joy:=false` to reuse existing nodes. The launch
passes the same geometry to the driver and joystick; when reusing a driver,
match its geometry explicitly. Steering is converted using
`yaw_rate=speed*tan(steering)/wheel_base`. Stop other motion publishers first.
Malformed joystick messages publish zero; loss of joystick messages is handled
by the driver's command timeout. The optional `joy` package supplies `joy_node`.

## Interfaces

The driver publishes `/wheel/odom` (`nav_msgs/msg/Odometry`) and dynamic TF
`odom -> base_link` from firmware posture. Linear velocity comes from rear-wheel
telemetry. Yaw rate is a wrapped difference of received headings over host ROS
time, since measured steering is unavailable. Serial latency affects this
estimate. The master launch adds no assumed sensor or localization transforms.
Topics and frame names can be configured in YAML; geometry and motion settings
are read at startup.

Stop motion before resetting firmware odometry:

```bash
ros2 topic pub --once /set_odometry geometry_msgs/msg/Pose2D \
  '{x: 1.0, y: 2.0, theta: 0.5}'
```

Or use `ros2 run mtrackercar set_odometry_demo --ros-args -p x:=1.0`.
Reset is sent once, without a separate acknowledgement; restart feedback after
a reset. See [protocol details](doc/protocol.md).

## Verification

```bash
colcon test --packages-select mtrackercar
colcon test-result --test-result-base build/mtrackercar --verbose
```

The portable protocol/kinematics test can also run without ROS:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include \
  src/communication/protocol.cpp test/test_protocol.cpp -o /tmp/test_car
/tmp/test_car
```

Portable tests cover car request layout, fragmented parsing, CRC rejection,
firmware reply layout, forward/reverse conversion, steering saturation, zero
speed and invalid commands. ROS launch and controller tests require ROS 2.
Physical driving and RViz require verification on the target hardware.

## Firmware migration and calibration

This driver requires the fixed-point CAR protocol: CMD=5 requests have LEN=20,
responses use CMD=5/LEN=22. Legacy CMD=3 CAR replies are rejected. The header
contains separate 8-bit ROBOT and SEQ fields; `robot_address` must be 0..255,
and replies must match both the robot and the current sequence. The old 256
address mask is no longer valid. The transport remains serial; nRF24 is not implemented.
See [the migration specification](doc/protocol.md) for exact layouts and signs.

The optional `cmd_vel` source retains speed in m/s and yaw rate in rad/s.
Normal command sources require `steering_mode: angle` (firmware calibrated
radians). Raw servo mode is exclusive to `command_source=calibration_raw`.
Voltage, optional current/power, motor output and power flags are exposed in
`/diagnostics`. Invalid current/power readings are marked `unavailable`.

Rear-wheel commands use `round(-requestedRearVelocityRadPerSec * 256)` on the wire.
Received velocity keeps the firmware physical sign (positive forward).
After rebuilding, restart the driver; verify physical forward/reverse motion
and agreement with measured odometry on the robot.

The `/set_regulator` service (`mtrackercar/srv/SetRegulator`) sets
`feedback_bandwidth`, `observer_bandwidth`, `damping`, and `inertia` together
using CMD=4. Values must be finite and positive. It stops drive and verifies
the returned status and active values. Changes take effect in RAM; use
`/save_configuration` separately for EEPROM persistence. There is no independent
read-only regulator query in the documented protocol. Stop motion publishers
and active calibration before tuning. See the regulator manual for commands.

Stop motion publishers before calibration. Set all four coefficients together
and call the service (coefficient ranges: c0 in [-1,1], c1..c3 in [-1000,1000]):

```bash
ros2 param set /mtrackercar steering_calibration '[0.0, 1.0, 0.0, 0.0]'
ros2 service call /set_steering_calibration std_srvs/srv/Trigger '{}'
```

The service first sends a disabled-drive command, then sends all coefficients
in one CMD=13 transaction and checks the returned status and active values.
Changing the parameter alone does not send calibration. Successful calibration
is active in RAM. To persist the entire firmware configuration, explicitly run:

```bash
ros2 service call /save_configuration std_srvs/srv/Trigger '{}'
```

This service stops drive, sends two consecutive CMD=8 requests (statuses 3 then
4), then polls CMD=8 with DATA[0]=1 until status 0. No motion command interrupts
the save sequence. Failure or timeout returns `success: false`; a timeout can
mean the outcome is unknown. Services run synchronously, with motion callbacks
paused during configuration (save completion deadline: 5 s). Commands are cleared
before configuration; resume by sending fresh commands after the service returns.
EEPROM writes are never requested automatically at startup.

A standalone PTY firmware emulator checks ROS odometry, motion wire signs,
calibration and uninterrupted EEPROM save:

```bash
python3 src/MTrackerCar-ROS/test/serial_migration_check.py \
  "$(ros2 pkg prefix mtrackercar)/lib/mtrackercar/mtrackercar_drv"
```

It emulates save status responses; actual EEPROM retention across firmware
reset still requires a hardware test. Set `ROS_LOG_DIR` to a writable directory
when running in a restricted environment.

## Steering identification from odom pose

Polska instrukcja obsługi: [manual Markdown](doc/calibration_manual_pl.md) ·
[wersja HTML do przeglądarki i wydruku](doc/calibration_manual_pl.html).

The `steering_calibration` program identifies the firmware's
`rawServo = c0 + c1*a + c2*a^2 + c3*a^3` mapping. It commands **uncalibrated raw
servo values**, records measured **x, y and heading**, and estimates each arc's
curvature from heading versus signed travelled distance. Physical steering is
`a = atan(wheel_base * curvature)`. The fit does not use wheel velocities or
infer steering from the commanded yaw rate. Odometry must report measured
motion (e.g. two rear encoders), not a pose integrated from these commands.
Encoder geometry errors and wheel slip remain sources of identification bias.
`wheel_base` is the effective longitudinal bicycle wheelbase, not rear track width.

Defaults in `config/calibration.yaml`: 9 raw settings from -0.60 to +0.60,
forward and reverse at 0.05 m/s, 3 full repetitions = **54 measured segments**.
Each segment includes a 1 s pause, 1.5 s moving settling interval and 8 s
measurement interval (nominal run about 9.5 minutes). Levels first ascend, then
descend, then use a seeded shuffled order. Directions are paired at each level
and their order alternates by repetition. Increase `repeats`/`raw_values` for
more data. Commanded settings are limited to [-0.8, 0.8]; firmware/mechanical
limits still determine which settings are suitable for your vehicle.

Stop the other controllers and use the dedicated launch:

```bash
ros2 launch mtrackercar calibration.launch.py enable_motion:=true
# In another sourced terminal, explicitly start the sequence:
ros2 service call /steering_calibration/start std_srvs/srv/Trigger '{}'
# Stop early and save a partial report:
ros2 service call /steering_calibration/cancel std_srvs/srv/Trigger '{}'
```

This launch runs the driver with `command_source=calibration_raw`, publishing
its measured odometry on `odom`. In this mode the driver subscribes only to
`calibration/raw_command` (`mtrackercar/msg/CarRawCommand`: rear rad/s and raw
servo), ignores `cmd_vel`, forces raw steering mode, independently caps calibration
speed at 0.10 m/s and retains its command watchdog. Normal driver launches keep
`command_source=steering_angle` by default, with optional `cmd_vel` compatibility.
Restart the driver in normal mode after calibration.
If reusing a driver (`start_driver:=false`), start it with the calibration source
and match wheel radius and odometry settings explicitly.

The input defaults to `odom` (`nav_msgs/Odometry`, using pose only). Set
`pose_topic:=...` for another pose source; `driver_odometry_topic:=...` separately
chooses where this launch publishes firmware odometry. For an existing
`geometry_msgs/Pose2D` stream use `pose_type:=pose2d` and a distinct topic, e.g.
`pose_topic:=measured_pose driver_odometry_topic:=firmware_odom`. Pose2D freshness
can only be checked by reception time because that message has no timestamp.
For Odometry, valid timestamps and stable reference/body frame names are required.
The pose must describe the rear-axle midpoint and forward heading; configure
`pose_offset_x/y` and `pose_yaw_offset` for a different tracked body origin.

Motion starts only after fresh pose feedback and confirmed serial diagnostics.
Pose loss, communication loss, odometry discontinuity, insufficient motion,
workspace radius/travel/time limits or cancellation stop the run. Default
workspace radius is 2 m from the initial pose, total travel budget 40 m and
time budget 900 s. A constant-radius arc is fitted after every segment; excessive
heading residuals abort identification. The final cubic must be observable,
monotonic over the measured range, span both sides of straight motion, agree
between forward/reverse, and pass fit and leave-one-servo-level-out error checks.
A poor experiment produces a failed report rather than usable coefficients.

Each run creates a unique directory under `calibration_results` containing:

- `poses.csv`: measured x/y/theta with source timestamps, raw setting, direction,
  repeat and step; `schedule.yaml`: all planned movements.
- `segments.csv`: identified angle, curvature, distance and heading residual per arc.
- `identification.yaml`: coefficients, measured ranges, fit diagnostics and settings.
- `steering_calibration.yaml`: ROS parameter file containing all four coefficients.
- `report.html`: standalone report with an embedded figure; `calibration.png` and
  `calibration.pdf`: trajectories, measured points and cubic, residuals, and
  forward/reverse means with repeatability bars. Partial/failed runs also produce
  a report and retain their recorded poses, without exporting accepted coefficients.

The result is **not automatically applied or written to EEPROM**. Inspect the
report, restrict normal operation to the measured angle range and transfer the
four coefficients through `/set_steering_calibration` as described above. Use
`/save_configuration` explicitly to persist the accepted calibration.

To repeat identification and reporting offline without driving:

```bash
ros2 run mtrackercar steering_calibration \
  --offline calibration_results/<run>/poses.csv --wheel-base 0.190 \
  --output-dir calibration_results/reanalysis
```

Offline replay uses the default quality thresholds and expects the already
rear-axle-corrected poses stored by acquisition; adjust the Python API settings
when reprocessing a run acquired with different quality thresholds.

## Left-controller experiments and automatic tuning

See the [Polish experiment/tuning manual](doc/regulator_experiments_pl.md)
([printable HTML](doc/regulator_experiments_pl.html)). By default the recorder
uses matched CMD=5 requests/replies from `regulator/drive_sample`: requested
velocity, measured velocity and motor output, with host receipt timestamps.
This mode scores velocity tracking without claiming access to internal error
or firmware-rate samples. Reference profiles include steps, ramps, sine, chirp and PRBS.
Bounded log-space coordinate search is followed by repeated holdout validation;
only an accepted candidate may be applied through CMD=4 and persisted via CMD=8.
Reports rank evaluated settings and label their acquisition source. By default,
the baseline is restored after validation and EEPROM is not written.
Optional `acquisition: firmware_buffer` uses actual CAR RAM channels 1/5/13/15:
rear velocity, output, position error and internal reference. That mode still
requires a verified TRACE provider and refuses to start without one.
A synthetic end-to-end demonstration
is available with `regulator_tuning --config <experiment.yaml> --mode tune --demo`.
