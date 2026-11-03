#!/usr/bin/env python3
"""Run bounded regulator tuning using CMD=5 telemetry or a firmware RAM adapter."""

import argparse
from dataclasses import asdict
from datetime import datetime
import math
from pathlib import Path
import threading
import time

import numpy as np
from regulator_experiment import (
    COLUMNS, evaluate, evaluate_telemetry, FIRMWARE_DEFAULTS, Profile, TELEMETRY_COLUMNS,
    TuneRunner, validate_parameters, write_capture, write_report)
import yaml


class SyntheticBackend:
    """Deterministic test plant; this is not a model of the MTracker ADRC."""

    synthetic = True

    def __init__(self):
        self.parameters = np.array(FIRMWARE_DEFAULTS)
        self.saved = False

    def stop(self):
        """No hardware is connected in demonstration mode."""

    def set_parameters(self, values):
        """Change only the synthetic response's parameter sensitivity."""
        self.parameters = np.asarray(values)

    def save(self):
        """Remember a simulated save without touching EEPROM."""
        self.saved = True

    def capture(self, profile):
        """Create synthetic measured velocity, controller error and output."""
        dt = 0.01
        times = np.arange(0, profile.duration + dt/2, dt)
        target = np.array([18.0, 90.0, 1.05, 0.0027])
        mismatch = float(np.sum(np.log(self.parameters / target)**2))
        tau = 0.035 + 0.5*mismatch
        speed = 0.0
        position_error = 0.0
        rows = []
        for index, t in enumerate(times):
            reference = profile.value(t)
            speed += (1-math.exp(-dt/tau))*(reference-speed)
            error = (reference-speed)/0.033
            position_error += error * dt
            output = float(np.clip(reference*5 + error*0.05, -1, 1))
            rows.append([index, float(t), reference, speed, position_error, output])
        return {'rows': rows, 'sample_period_s': dt, 'overflow': False}


class RosBufferBackend:
    """ROS client for a verified firmware-buffer adapter and existing driver APIs.

    No serial command numbers are inferred here. The separate capture provider
    must implement the firmware-specific START/STOP/READ operations. Without that
    provider, preflight fails before writing regulator parameters or moving.
    """

    def __init__(self, node, settings):
        from diagnostic_msgs.msg import DiagnosticArray
        from mtrackercar.srv import ControllerCapture, SetRegulator
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float64
        from std_srvs.srv import Trigger
        self.node, self.settings = node, settings
        self.cancelled = threading.Event()
        self.Float64, self.Capture = Float64, ControllerCapture
        self.Regulator, self.Trigger = SetRegulator, Trigger
        self.publisher = node.create_publisher(Float64, 'regulator/reference_velocity', 1)
        self.regulator = node.create_client(SetRegulator, 'set_regulator')
        self.capture_service = node.create_client(ControllerCapture, 'controller_capture')
        self.save_service = node.create_client(Trigger, 'save_configuration')
        self.feedback = None
        self.feedback_received = 0.0
        self.serial_ok = False
        self.serial_received = 0.0
        self.origin = None
        self.total_distance = 0.0
        self.started = None
        self.fault = None
        self.subscriptions = [
            node.create_subscription(Odometry, 'regulator/odom', self.on_odometry,
                                     qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)]

    def on_odometry(self, message):
        """Monitor fresh physical odometry while the firmware buffers fine samples."""
        p = message.pose.pose.position
        speed = message.twist.twist.linear.x
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        age = self.node.get_clock().now().nanoseconds*1e-9 - stamp
        if not all(math.isfinite(x) for x in (p.x, p.y, speed, age)):
            self.fault = 'Nonfinite odometry'
            return
        if not -0.05 <= age <= self.settings['feedback_timeout_s'] or stamp <= 0:
            self.fault = 'Stale odometry timestamp'
            return
        if self.started is not None and self.feedback is not None:
            self.total_distance += math.hypot(p.x-self.feedback[0], p.y-self.feedback[1])
        self.feedback = (p.x, p.y, speed)
        self.feedback_received = time.monotonic()

    def on_diagnostics(self, message):
        """Track driver-confirmed serial communication independently of captures."""
        for status in message.status:
            if status.name == 'mtrackercar/serial':
                self.serial_ok = status.level == status.OK
                self.serial_received = time.monotonic()

    def progress(self, trial):
        """Publish each completed trial so a long search remains observable."""
        from std_msgs.msg import String
        detail = (f'score={trial["metrics"]["score"]:.5f}' if trial['accepted']
                  else ('REJECTED (continuing): ' if trial.get('quality_rejected') else 'FAILED: ')
                  + trial.get('error', 'unknown error'))
        message = f'Trial {trial["trial"]} {trial["phase"]}: {detail}'
        self.node.get_logger().info(message)
        if hasattr(self, 'status_publisher'):
            self.status_publisher.publish(String(data=message))

    def preflight(self):
        """Refuse to run until the buffer provider and motion feedback are ready."""
        for client in (self.capture_service, self.regulator):
            if not client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError('Required service unavailable: ' + client.srv_name +
                                   '; firmware buffer adapter/specification is required')
        persist = self.settings.get('persist_best', False)
        if persist and not self.save_service.wait_for_service(2.0):
            raise RuntimeError('EEPROM save service unavailable')
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError('Start driver with command_source=regulator_tuning')
        self.check_feedback()
        self.origin = self.feedback[:2]
        self.started = time.monotonic()

    def check_feedback(self):
        """Abort on cancellation, serial loss, excessive velocity or workspace exit."""
        if self.cancelled.is_set():
            raise RuntimeError('Cancelled by operator')
        if self.fault:
            raise RuntimeError(self.fault)
        feedback_age = time.monotonic() - self.feedback_received
        if self.feedback is None or feedback_age > self.settings['feedback_timeout_s']:
            raise RuntimeError('Fresh odometry feedback required')
        if not self.serial_ok or time.monotonic()-self.serial_received > 2.5:
            raise RuntimeError('Serial feedback lost')
        x, y, speed = self.feedback
        if abs(speed) > self.settings['max_speed_m_s']:
            raise RuntimeError('Measured speed limit exceeded')
        radius = math.hypot(x-self.origin[0], y-self.origin[1]) if self.origin else 0
        if radius > self.settings['max_radius_m']:
            raise RuntimeError('Experiment workspace radius exceeded')
        if self.total_distance > self.settings.get('max_total_distance_m', 10.0):
            raise RuntimeError('Experiment travel budget exceeded')
        if (self.started is not None and
                time.monotonic()-self.started > self.settings.get('max_duration_s', 600.0)):
            raise RuntimeError('Experiment time budget exceeded')

    def call(self, client, request, timeout=5.0):
        """Wait in the worker thread while the ROS executor handles responses."""
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda unused: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise RuntimeError('Service timeout; result unknown: ' + client.srv_name)
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def stop(self):
        """Immediately publish a zero velocity on the exclusive tuning input."""
        self.publisher.publish(self.Float64(data=0.0))

    def wait_stopped(self):
        """Allow a bounded recovery pause after a rejected settling measurement."""
        deadline = time.monotonic()+5.0
        stable_since = None
        while time.monotonic() < deadline:
            self.stop()
            self.check_feedback()
            if abs(self.feedback[2]) <= 0.015:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic()-stable_since >= 0.4:
                    return
            else:
                stable_since = None
            self.cancelled.wait(0.02)
        raise RuntimeError('Robot still moving after 5 s recovery stop; tuning aborted')

    def set_parameters(self, values):
        """Apply all four values through the driver's checked CMD=4 transaction."""
        request = self.Regulator.Request()
        (request.feedback_bandwidth, request.observer_bandwidth,
         request.damping, request.inertia) = map(float, values)
        self.call(self.regulator, request)
        # Configuration pauses periodic telemetry; do not treat that known pause
        # as a lost stream at the beginning of the next capture.
        completed = time.monotonic()
        while time.monotonic()-completed < self.settings['feedback_timeout_s']+0.5:
            if self.feedback_received > completed:
                return
            time.sleep(0.01)
        raise RuntimeError('No fresh telemetry after regulator update; result unknown')

    def restore_parameters(self, values):
        """Wait for fresh confirmed feedback after reconnect before one rollback."""
        self.stop()
        started = time.monotonic()
        deadline = started+5.0
        while time.monotonic() < deadline:
            if (self.feedback_received > started and self.serial_ok and
                    time.monotonic()-self.serial_received <= 2.5):
                self.set_parameters(values)
                return
            time.sleep(0.02)
        raise RuntimeError('Baseline restoration unconfirmed: serial feedback did not recover')

    def save(self):
        """Persist only through the driver-confirmed two-step EEPROM operation."""
        if self.cancelled.is_set():
            raise RuntimeError('Save cancelled before request')
        self.call(self.save_service, self.Trigger.Request(), timeout=8.0)

    def capture(self, profile):
        """Run the reference and read paged samples from the firmware capture."""
        self.check_feedback()
        request = self.Capture.Request(action=self.Capture.Request.START,
                                       duration_s=profile.duration)
        started = self.call(self.capture_service, request)
        identity = started.capture_id
        if not identity:
            raise RuntimeError('Capture provider did not identify the firmware run')
        try:
            dt = started.sample_period_s
            age = started.capture_age_s
            valid_period = math.isfinite(dt) and dt > 0
            valid_age = math.isfinite(age) and 0 <= age < profile.quiet/2
            if not (valid_period and valid_age):
                raise RuntimeError('Invalid capture period or trigger age')
            if started.capacity < math.ceil(profile.duration/dt):
                raise RuntimeError('Requested experiment exceeds firmware buffer capacity')
            origin = time.monotonic() - age
            interval = 1.0/self.settings['command_rate_hz']
            while time.monotonic()-origin < profile.duration:
                self.check_feedback()
                elapsed = time.monotonic()-origin
                self.publisher.publish(self.Float64(data=profile.value(elapsed)))
                self.cancelled.wait(interval)
            self.stop()
            stopped = self.call(self.capture_service, self.Capture.Request(
                action=self.Capture.Request.STOP, capture_id=identity))
            if stopped.capture_id != identity or stopped.overflow or not stopped.complete:
                raise RuntimeError('Firmware capture is incomplete or overflowed')
            total = stopped.total_samples
            if total == 0 or total > started.capacity:
                raise RuntimeError('Invalid firmware capture sample count')
            rows = []
            while len(rows) < total:
                if self.cancelled.is_set():
                    raise RuntimeError('Cancelled during buffer download')
                response = self.call(self.capture_service, self.Capture.Request(
                    action=self.Capture.Request.READ, capture_id=identity,
                    offset=len(rows), max_samples=min(256, total-len(rows))))
                if (response.capture_id != identity or response.total_samples != total or
                        response.overflow or not response.complete or not response.samples or
                        len(response.samples) > min(256, total-len(rows))):
                    raise RuntimeError('Inconsistent firmware capture page')
                for sample in response.samples:
                    if sample.index != len(rows):
                        raise RuntimeError('Firmware capture page index mismatch')
                    rows.append([sample.index, sample.time_s, sample.reference_velocity_m_s,
                                 sample.velocity_m_s, sample.left_position_error_rad,
                                 sample.left_output])
            return {'rows': rows, 'sample_period_s': dt, 'overflow': False}
        finally:
            self.stop()
            # STOP is idempotent in the adapter contract; also clean up aborted runs.
            try:
                self.call(self.capture_service, self.Capture.Request(
                    action=self.Capture.Request.STOP, capture_id=identity))
            except Exception:
                pass


class TelemetrySyntheticBackend(SyntheticBackend):
    """Exercise the telemetry objective and CSV schema without hardware."""

    acquisition = 'cmd5_telemetry'
    columns = TELEMETRY_COLUMNS
    evaluate = staticmethod(evaluate_telemetry)

    def capture(self, profile):
        """Drop internal position error; preserve actual synthetic motor output."""
        capture = super().capture(profile)
        capture['rows'] = [row[:4] + [row[5]] for row in capture['rows']]
        return capture


class RosTelemetryBackend(RosBufferBackend):
    """Tune from matched CMD=5 requests/replies, without a RAM capture adapter."""

    acquisition = 'cmd5_telemetry'
    columns = TELEMETRY_COLUMNS
    evaluate = staticmethod(evaluate_telemetry)

    def __init__(self, node, settings):
        from mtrackercar.msg import DriveSample
        super().__init__(node, settings)
        self.capture_lock = threading.Lock()
        self.capture_origin = None
        self.capture_rows = []
        self.sample_received = 0.0
        self.last_stamp = None
        self.subscriptions.append(node.create_subscription(
            DriveSample, 'regulator/drive_sample', self.on_sample, 100))

    def on_sample(self, sample):
        """Keep request echo and measured velocity together in the same sample."""
        values = (sample.stamp_s, sample.requested_velocity_m_s,
                  sample.measured_velocity_m_s, sample.motor_output)
        age = self.node.get_clock().now().nanoseconds*1e-9-sample.stamp_s
        if not all(math.isfinite(v) for v in values) or not -0.05 <= age <= 0.15:
            self.fault = 'Invalid or stale drive sample'
            return
        if self.last_stamp is not None and sample.stamp_s <= self.last_stamp:
            self.fault = 'Drive telemetry clock moved backwards or repeated'
            return
        self.last_stamp = sample.stamp_s
        self.sample_received = time.monotonic()
        with self.capture_lock:
            if self.capture_origin is not None and sample.stamp_s >= self.capture_origin:
                self.capture_rows.append([len(self.capture_rows),
                                          sample.stamp_s-self.capture_origin, *values[1:]])

    def preflight(self):
        """Check exclusive command input, baseline services and fresh telemetry."""
        if not self.regulator.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('set_regulator service unavailable')
        if self.settings.get('persist_best') and not self.save_service.wait_for_service(2.0):
            raise RuntimeError('EEPROM save service unavailable')
        if self.publisher.get_subscription_count() != 1:
            raise RuntimeError('Exactly one driver with command_source=regulator_tuning required')
        if (self.node.count_publishers('regulator/reference_velocity') != 1 or
                self.node.count_publishers('regulator/drive_sample') != 1):
            raise RuntimeError('Multiple tuning publishers or drive telemetry sources detected')
        self.check_feedback()
        self.origin = self.feedback[:2]
        self.started = time.monotonic()

    def check_feedback(self):
        """Require continuous fast telemetry as well as physical odometry."""
        super().check_feedback()
        if time.monotonic()-self.sample_received > 0.15:
            raise RuntimeError('Fresh CMD=5 drive samples required')

    def capture(self, profile):
        """Apply a bounded profile and record only the driver's real replies."""
        self.check_feedback()
        origin = time.monotonic()
        with self.capture_lock:
            self.capture_rows = []
            self.capture_origin = self.node.get_clock().now().nanoseconds*1e-9
        try:
            while time.monotonic()-origin < profile.duration:
                self.check_feedback()
                self.publisher.publish(self.Float64(data=profile.value(time.monotonic()-origin)))
                self.cancelled.wait(1.0/self.settings['command_rate_hz'])
        finally:
            self.stop()
            with self.capture_lock:
                self.capture_origin = None
                rows = list(self.capture_rows)
        if len(rows) < 2:
            raise RuntimeError('No CMD=5 telemetry captured')
        period = float(np.median(np.diff(np.asarray(rows)[:, 1])))
        return {'rows': rows, 'sample_period_s': period, 'overflow': False}


def validate_settings(settings):
    """Reject invalid motion/optimization settings before any hardware operation."""
    for key in ('enable_motion', 'baseline_confirmed', 'apply_best', 'persist_best'):
        if type(settings.get(key)) is not bool:
            raise ValueError(f'{key} must be a YAML boolean')
    for key in ('max_trials', 'validation_repeats'):
        if type(settings.get(key)) is not int:
            raise ValueError(f'{key} must be an integer')
    validate_parameters(settings['baseline'], settings['bounds'])
    Profile(**settings['training']).validate()
    Profile(**settings['validation']).validate()
    positive = ('wheel_radius', 'command_rate_hz', 'max_speed_m_s',
                'feedback_timeout_s', 'max_radius_m')
    if any(not math.isfinite(settings[k]) or settings[k] <= 0 for k in positive):
        raise ValueError('Experiment limits and geometry must be finite and positive')
    if settings['command_rate_hz'] > 100:
        raise ValueError('Command rate must not exceed 100 Hz')
    if settings['persist_best'] and not settings['apply_best']:
        raise ValueError('persist_best requires apply_best')
    if settings.get('acquisition', 'firmware_buffer') not in ('cmd5_telemetry', 'firmware_buffer'):
        raise ValueError('Unknown acquisition backend')
    for key, default in (('max_duration_s', 600.0), ('max_total_distance_m', 10.0)):
        if not math.isfinite(settings.get(key, default)) or settings.get(key, default) <= 0:
            raise ValueError('Experiment time and distance limits must be positive')
    if settings['max_trials'] < 1 or settings['validation_repeats'] < 2:
        raise ValueError('Invalid trial or validation budget')
    if not 0 <= settings['min_improvement'] < 1:
        raise ValueError('Invalid minimum improvement')
    amplitude = max(settings['training']['amplitude'], settings['validation']['amplitude'])
    if amplitude >= settings['max_speed_m_s']:
        raise ValueError('Measured speed limit must exceed requested profile amplitude')


def run_experiment(backend, settings, directory, mode):
    """Execute one recording or a complete search and always write a report."""
    validate_settings(settings)
    training = Profile(**settings['training']).validate()
    validation = Profile(**settings['validation']).validate()
    runner = TuneRunner(backend, directory, settings['baseline'], settings['bounds'],
                        settings['max_trials'], settings['wheel_radius'],
                        settings['validation_repeats'], settings['min_improvement'])
    try:
        if mode == 'record':
            backend.stop()
            capture = backend.capture(training)
            write_capture(directory / 'trial_000.csv', capture['rows'],
                          getattr(backend, 'columns', COLUMNS))
            trial = {'trial': 0, 'phase': 'record', 'parameters': None,
                     'profile': asdict(training), 'capture': 'trial_000.csv',
                     'sample_period_s': capture['sample_period_s'], 'accepted': False}
            runner.trials.append(trial)
            if capture.get('overflow', False):
                raise RuntimeError('Firmware buffer overflow')
            scorer = getattr(backend, 'evaluate', evaluate)
            trial['metrics'] = scorer(capture['rows'], training, capture['sample_period_s'],
                                      settings['wheel_radius'])
            trial['accepted'] = True
            runner.checkpoint({'mode': 'record', 'applied': False, 'persisted': False})
        else:
            if not settings.get('baseline_confirmed', False):
                raise RuntimeError('Confirm a known-working baseline before automatic tuning')
            runner.run(training, validation, settings['apply_best'], settings['persist_best'])
    except Exception as error:
        if not (directory / 'tuning.yaml').exists():
            runner.checkpoint({'error': str(error), 'applied': False, 'persisted': False})
        raise
    finally:
        backend.stop()
        if (directory / 'tuning.yaml').exists():
            write_report(directory)


def main():
    """Select a hardware recording/search or a clearly marked synthetic example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--mode', choices=['record', 'tune'], default='record')
    parser.add_argument('--demo', action='store_true', help='Synthetic plant; no robot I/O')
    args, ros_args = parser.parse_known_args()
    settings = yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    validate_settings(settings)
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    directory = Path(settings['output_dir']).expanduser() / stamp
    if args.demo:
        settings['baseline_confirmed'] = True
        backend = (TelemetrySyntheticBackend() if settings.get('acquisition') == 'cmd5_telemetry'
                   else SyntheticBackend())
        run_experiment(backend, settings, directory, args.mode)
        document = yaml.safe_load((directory / 'tuning.yaml').read_text())
        document['synthetic'] = True
        (directory / 'tuning.yaml').write_text(yaml.safe_dump(document, sort_keys=False))
        write_report(directory)
        print(directory / 'report.html')
        return
    run_ros(settings, directory, args.mode, ros_args)


def run_ros(settings, directory, mode, arguments):
    """Offer explicit start/cancel services and run experiments in a worker thread."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=arguments, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('regulator_tuning')
    backend_type = (RosTelemetryBackend if settings.get('acquisition') == 'cmd5_telemetry'
                    else RosBufferBackend)
    backend = backend_type(node, settings)
    status = node.create_publisher(String, 'regulator/status', 10)
    backend.status_publisher = status
    worker = [None]

    def execute():
        try:
            backend.preflight()
            run_experiment(backend, settings, directory, mode)
            message = f'Completed; report={directory / "report.html"}'
        except Exception as error:
            message = 'Failed: ' + str(error)
        finally:
            backend.stop()
        node.get_logger().info(message)
        status.publish(String(data=message))

    def start(request, response):
        del request
        if not settings['enable_motion']:
            response.message = 'Set enable_motion: true in the experiment config'
        elif worker[0] is not None:
            response.message = 'This process already started a run; restart for a new run'
        else:
            backend.cancelled.clear()
            worker[0] = threading.Thread(target=execute, daemon=True)
            worker[0].start()
            response.success = True
            response.message = 'Preflight started; track /regulator/status for execution outcome'
        return response

    def cancel(request, response):
        del request
        backend.cancelled.set()
        backend.stop()
        response.success = True
        response.message = 'Cancellation requested; check status for rollback outcome'
        return response

    services = [node.create_service(Trigger, '~/start', start),
                node.create_service(Trigger, '~/cancel', cancel)]
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        backend.cancelled.set()
        if rclpy.ok():
            backend.stop()
            # Keep responses flowing while the worker stops and restores baseline.
            deadline = time.monotonic()+20.0
            while worker[0] is not None and worker[0].is_alive() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if worker[0] is not None and worker[0].is_alive():
                node.get_logger().error('Shutdown timeout; baseline restoration unconfirmed')
        for service in services:
            node.destroy_service(service)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
