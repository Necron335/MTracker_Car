#!/usr/bin/env python3
"""Monitor measured supply voltage and optionally request an orderly PC shutdown."""

import math
import subprocess
import time


class VoltageGuard:
    """Debounced undervoltage with recovery hysteresis and stale-data rejection."""

    def __init__(self, cutoff, hysteresis, hold, countdown, timeout):
        values = (cutoff, hysteresis, hold, countdown, timeout)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError('Voltage threshold and timing settings must be positive')
        self.cutoff, self.hysteresis = cutoff, hysteresis
        self.hold, self.countdown, self.timeout = hold, countdown, timeout
        self.received = None
        self.low_since = None
        self.armed_at = None
        self.voltage = None

    def invalidate(self):
        """A missing measurement is unknown power state, never zero volts."""
        self.received = self.low_since = self.armed_at = None

    def sample(self, voltage, now):
        """Only continuous, finite measurements may sustain a shutdown decision."""
        if not math.isfinite(voltage) or voltage <= 0:
            self.invalidate()
            return
        if self.received is None or now-self.received > self.timeout:
            self.low_since = self.armed_at = None
        self.voltage, self.received = voltage, now
        if voltage >= self.cutoff+self.hysteresis:
            self.low_since = self.armed_at = None
        elif voltage < self.cutoff and self.low_since is None:
            self.low_since = now

    def state(self, now):
        """Return unknown, normal, low, countdown or shutdown."""
        if self.received is None or now-self.received > self.timeout:
            self.invalidate()
            return 'unknown'
        if self.low_since is None:
            return 'normal'
        if now-self.low_since < self.hold:
            return 'low'
        if self.armed_at is None:
            self.armed_at = now
        return 'shutdown' if now-self.armed_at >= self.countdown else 'countdown'


def main():
    """Consume driver diagnostics; dry-run is the default and never powers off."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node('pc_power_monitor')
    cutoff = node.declare_parameter('shutdown_voltage', 0.0).value
    hysteresis = node.declare_parameter('hysteresis_voltage', 0.3).value
    hold = node.declare_parameter('low_duration_s', 10.0).value
    countdown = node.declare_parameter('shutdown_delay_s', 15.0).value
    timeout = node.declare_parameter('measurement_timeout_s', 2.5).value
    enabled = node.declare_parameter('enable_shutdown', False).value
    source = node.declare_parameter('diagnostic_name', 'mtrackercar/serial').value
    device = node.declare_parameter('hardware_id', '/dev/ttyUSB0').value
    if not device:
        raise ValueError('Set hardware_id to identify the intended voltage source')
    if enabled and cutoff <= 0:
        raise ValueError('Configure shutdown_voltage before enabling shutdown')
    guard = VoltageGuard(cutoff, hysteresis, hold, countdown, timeout) if cutoff > 0 else None
    status = node.create_publisher(String, 'power_monitor/status', 10)
    last_state = [None]
    attempted = [False]
    observed_voltage = [None]

    def receive(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        age = node.get_clock().now().nanoseconds*1e-9-stamp
        matches = [s for s in message.status if s.name == source and s.hardware_id == device]
        if not matches:
            return
        if len(matches) != 1 or not -0.05 <= age <= timeout or stamp <= 0:
            if guard:
                guard.invalidate()
            return
        item = matches[0]
        try:
            voltage = float(next(v.value for v in item.values if v.key == 'voltage_V'))
            if item.level != item.OK:
                raise ValueError('Unconfirmed communication')
            if not math.isfinite(voltage) or voltage <= 0:
                raise ValueError('Invalid voltage')
            observed_voltage[0] = voltage
            if guard:
                guard.sample(voltage, time.monotonic())
        except (ValueError, StopIteration):
            if guard:
                guard.invalidate()

    def update():
        state = guard.state(time.monotonic()) if guard else 'unconfigured'
        voltage = guard.voltage if guard else observed_voltage[0]
        text = f'{state}; voltage_V={voltage}; enable_shutdown={enabled}'
        status.publish(String(data=text))
        if state != last_state[0]:
            node.get_logger().info(text)
            last_state[0] = state
        if state in ('normal', 'unknown'):
            attempted[0] = False
        if state == 'shutdown' and not attempted[0]:
            attempted[0] = True
            if not enabled:
                node.get_logger().warning('DRY RUN: would request PC shutdown now')
                return
            try:
                subprocess.run(['/usr/bin/sudo', '-n', '/usr/bin/systemctl', 'poweroff'],
                               check=True, timeout=10, capture_output=True, text=True)
                node.get_logger().warning('PC shutdown requested')
            except (subprocess.SubprocessError, OSError) as error:
                node.get_logger().error(f'PC shutdown request failed: {error}')

    subscription = node.create_subscription(DiagnosticArray, '/diagnostics', receive, 10)
    timer = node.create_timer(0.5, update)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
