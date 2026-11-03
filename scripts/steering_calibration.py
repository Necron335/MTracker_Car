#!/usr/bin/env python3
"""Acquire forward/reverse raw-servo sweeps and identify a steering polynomial."""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

from calibration_identification import (
    estimate_segment, fit_calibration, identify_csv, make_schedule, write_report, write_result)


def main():
    """Run ROS acquisition or repeat the fit from a recorded CSV without ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', help='Recorded poses.csv to identify without driving')
    parser.add_argument('--wheel-base', type=float, default=0.190)
    parser.add_argument('--output-dir', default='calibration_results')
    arguments, ros_arguments = parser.parse_known_args()
    if arguments.offline:
        run_id = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        directory = Path(arguments.output_dir) / run_id
        directory.mkdir(parents=True, exist_ok=False)
        import shutil
        shutil.copyfile(arguments.offline, directory / 'poses.csv')
        metadata = {'source': str(arguments.offline), 'wheel_base': arguments.wheel_base}
        segments = []
        try:
            segments, result = identify_csv(directory / 'poses.csv', arguments.wheel_base)
            write_result(directory, segments, result, metadata)
            write_report(directory, segments, result, metadata)
        except Exception as error:
            write_report(directory, segments, None, metadata, str(error))
            raise
        print(directory / 'report.html')
        return
    run_ros(ros_arguments)


def run_ros(arguments):
    """Keep ROS dependencies out of the offline identification path."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Pose2D
    from mtrackercar.msg import CarRawCommand
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class CalibrationNode(Node):
        """Explicitly started acquisition with pose, serial and travel watchdogs."""

        def __init__(self):
            super().__init__('steering_calibration')
            defaults = {
                'enable_motion': False, 'pose_topic': 'odom', 'pose_type': 'odometry',
                'wheel_base': 0.190, 'wheel_radius': 0.033, 'speed': 0.05,
                'raw_values': [-0.60, -0.45, -0.30, -0.15, 0.0, 0.15, 0.30, 0.45, 0.60],
                'repeats': 3, 'seed': 7, 'pause_duration': 1.0, 'settle_duration': 1.5,
                'measure_duration': 8.0, 'pose_timeout': 0.3, 'min_distance': 0.15,
                'max_radius': 2.0, 'max_total_distance': 40.0, 'max_duration': 900.0,
                'max_yaw_rms': 0.025, 'max_fit_rmse': 0.04, 'max_direction_difference': 0.06,
                'pose_offset_x': 0.0, 'pose_offset_y': 0.0, 'pose_yaw_offset': 0.0,
                'output_dir': 'calibration_results'}
            self.settings = {key: self.declare_parameter(key, value).value
                             for key, value in defaults.items()}
            self.validate()
            self.schedule = make_schedule(self.settings['raw_values'],
                                          self.settings['repeats'], self.settings['seed'])
            self.publisher = self.create_publisher(CarRawCommand, 'calibration/raw_command', 1)
            self.status_publisher = self.create_publisher(String, 'calibration/status', 10)
            msg_type = Odometry if self.settings['pose_type'] == 'odometry' else Pose2D
            self.pose_subscription = self.create_subscription(
                msg_type, self.settings['pose_topic'], self.on_pose, qos_profile_sensor_data)
            self.diagnostics_subscription = self.create_subscription(
                DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
            self.start_service = self.create_service(Trigger, '~/start', self.start)
            self.cancel_service = self.create_service(Trigger, '~/cancel', self.cancel)
            self.timer = self.create_timer(0.05, self.update)
            self.active = False
            self.pose = None
            self.pose_received = 0.0
            self.driver_received = 0.0
            self.driver_ok = False
            self.frame = None
            self.stream = None
            self.directory = None
            self.segments = []
            self.samples = []
            self.total_distance = 0.0
            self.status(f'Ready: {len(self.schedule)} segments; '
                        'call ~/start with enable_motion=true')

        def validate(self):
            s = self.settings
            positive = ('wheel_base', 'wheel_radius', 'speed', 'pause_duration',
                        'settle_duration', 'measure_duration', 'pose_timeout', 'min_distance',
                        'max_radius', 'max_total_distance', 'max_duration', 'max_yaw_rms',
                        'max_fit_rmse', 'max_direction_difference')
            if any(not math.isfinite(s[k]) or s[k] <= 0 for k in positive):
                raise ValueError('Calibration geometry, durations and limits must be positive')
            if s['speed'] > 0.10 or s['measure_duration'] * s['speed'] < s['min_distance'] * 1.2:
                raise ValueError('Use speed <=0.10 m/s and enough measurement travel')
            if s['pose_type'] not in ('odometry', 'pose2d'):
                raise ValueError('pose_type must be odometry or pose2d')
            offsets = ('pose_offset_x', 'pose_offset_y', 'pose_yaw_offset')
            if not all(math.isfinite(s[k]) for k in offsets):
                raise ValueError('Invalid pose offset')

        def status(self, text):
            self.get_logger().info(text)
            message = String()
            message.data = text
            if rclpy.ok():
                self.status_publisher.publish(message)

        def on_diagnostics(self, message):
            for item in message.status:
                if item.name == 'mtrackercar/serial':
                    self.driver_ok = item.level == item.OK
                    self.driver_received = time.monotonic()

        def on_pose(self, message):
            try:
                if self.settings['pose_type'] == 'odometry':
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if stamp <= 0 or not -0.05 <= now - stamp <= self.settings['pose_timeout']:
                        raise ValueError('Odometry timestamp is stale, zero or in the future')
                    frame = (message.header.frame_id, message.child_frame_id)
                    if not all(frame):
                        raise ValueError('Odometry must identify its reference and body frames')
                    if self.active and self.frame != frame:
                        raise ValueError('Odometry frames changed during identification')
                    self.frame = frame
                    p = message.pose.pose
                    x, y = p.position.x, p.position.y
                    q = p.orientation
                    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
                        raise ValueError('Invalid odometry quaternion')
                    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                else:
                    # Pose2D has no source timestamp: freshness is reception-time only.
                    stamp = self.get_clock().now().nanoseconds * 1e-9
                    x, y, yaw = message.x, message.y, message.theta
                    self.frame = ('pose2d', 'configured_body')
                yaw -= self.settings['pose_yaw_offset']
                offset_x = self.settings['pose_offset_x']
                offset_y = self.settings['pose_offset_y']
                x -= math.cos(yaw)*offset_x - math.sin(yaw)*offset_y
                y -= math.sin(yaw)*offset_x + math.cos(yaw)*offset_y
                pose = (stamp, x, y, yaw)
                if not all(math.isfinite(v) for v in pose):
                    raise ValueError('Nonfinite pose')
                if self.pose and stamp <= self.pose[0]:
                    if self.active and stamp < self.pose[0]:
                        raise ValueError('Odometry clock moved backwards')
                    return
                if self.active:
                    if self.pose:
                        dt = stamp - self.pose[0]
                        distance = math.hypot(x - self.pose[1], y - self.pose[2])
                        yaw_change = math.atan2(math.sin(yaw - self.pose[3]),
                                                math.cos(yaw - self.pose[3]))
                        if abs(yaw_change) > 0.3:
                            raise ValueError('Orientation discontinuity')
                        if dt > self.settings['pose_timeout'] or distance > 0.30*dt + 0.02:
                            raise ValueError('Missing odometry or position discontinuity')
                        self.total_distance += distance
                    radius = math.hypot(x - self.origin[0], y - self.origin[1])
                    if radius > self.settings['max_radius']:
                        raise ValueError('Calibration workspace radius exceeded')
                    if self.total_distance > self.settings['max_total_distance']:
                        raise ValueError('Calibration travel budget exceeded')
                    if self.phase == 'measure':
                        self.samples.append(pose)
                        item = self.schedule[self.step]
                        self.writer.writerow({
                            'step': self.step, 'repeat': item['repeat'],
                            'raw': item['raw'], 'direction': item['direction'],
                            'stamp': stamp, 'x': x, 'y': y, 'theta': yaw})
                        self.stream.flush()
                self.pose = pose
                self.pose_received = time.monotonic()
            except Exception as error:
                if self.active:
                    self.finish(str(error))
                else:
                    self.pose = None

        def send(self, speed=0.0, raw=0.0):
            message = CarRawCommand()
            message.rear_velocity_rad_s = speed / self.settings['wheel_radius']
            message.raw_servo = raw
            self.publisher.publish(message)

        def start(self, request, response):
            del request
            if self.active:
                response.message = 'Calibration is already running'
                return response
            current = time.monotonic()
            if not self.settings['enable_motion']:
                response.message = 'Set enable_motion=true at startup to authorize motion'
            elif self.pose is None or current - self.pose_received > self.settings['pose_timeout']:
                response.message = 'Fresh odom pose is required'
            elif not self.driver_ok or current - self.driver_received > 2.5:
                response.message = 'Confirmed driver serial communication is required'
            elif self.publisher.get_subscription_count() == 0:
                response.message = 'Start driver with command_source=calibration_raw'
            else:
                try:
                    run_id = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
                    self.directory = Path(self.settings['output_dir']).expanduser() / run_id
                    self.directory.mkdir(parents=True, exist_ok=False)
                    self.stream = (self.directory / 'poses.csv').open(
                        'w', newline='', encoding='utf-8')
                    self.writer = csv.DictWriter(self.stream, fieldnames=[
                        'step', 'repeat', 'raw', 'direction', 'stamp', 'x', 'y', 'theta'])
                    self.writer.writeheader()
                    import yaml
                    (self.directory / 'schedule.yaml').write_text(
                        yaml.safe_dump(self.schedule, sort_keys=False), encoding='utf-8')
                    self.segments, self.samples = [], []
                    self.origin = self.pose[1:3]
                    self.total_distance = 0.0
                    self.step = 0
                    self.phase = 'pause'
                    self.phase_start = self.started = current
                    self.active = True
                    self.send()
                    response.success = True
                    response.message = (f'Started {len(self.schedule)} segments; '
                                        f'output={self.directory}')
                    self.status(response.message)
                except Exception as error:
                    response.message = str(error)
                    if self.stream:
                        self.stream.close()
                        self.stream = None
            return response

        def cancel(self, request, response):
            del request
            if self.active:
                self.finish('Cancelled by operator')
                response.success = True
                response.message = 'Stopped; partial measurements and report saved'
            else:
                self.send()
                response.success = True
                response.message = 'Already stopped'
            return response

        def update(self):
            if not self.active:
                return
            current = time.monotonic()
            try:
                if current - self.pose_received > self.settings['pose_timeout']:
                    raise ValueError('Pose feedback lost')
                if not self.driver_ok or current - self.driver_received > 2.5:
                    raise ValueError('Driver serial feedback lost')
                if current - self.started > self.settings['max_duration']:
                    raise ValueError('Calibration duration budget exceeded')
                item = self.schedule[self.step]
                duration = self.settings[{'pause': 'pause_duration', 'settle': 'settle_duration',
                                          'measure': 'measure_duration'}[self.phase]]
                if current - self.phase_start >= duration:
                    if self.phase == 'pause':
                        self.phase = 'settle'
                    elif self.phase == 'settle':
                        self.phase = 'measure'
                        self.samples = []
                    else:
                        self.send()
                        segment = estimate_segment(
                            self.samples, item['raw'], item['direction'],
                            self.settings['wheel_base'],
                            self.settings['min_distance'], self.settings['pose_timeout'],
                            self.settings['max_yaw_rms'])
                        segment['step'] = self.step
                        segment['repeat'] = item['repeat']
                        self.segments.append(segment)
                        self.step += 1
                        if self.step == len(self.schedule):
                            self.finish()
                            return
                        self.phase = 'pause'
                    self.phase_start = current
                    item = self.schedule[self.step]
                    self.status(f'{self.step + 1}/{len(self.schedule)} {self.phase}: '
                                f'raw={item["raw"]:+.2f} direction={item["direction"]:+d}')
                speed = (0.0 if self.phase == 'pause' else
                         item['direction'] * self.settings['speed'])
                self.send(speed, item['raw'])
            except Exception as error:
                self.finish(str(error))

        def finish(self, error=None):
            self.active = False
            if rclpy.ok():
                self.send()
            if self.stream:
                self.stream.close()
                self.stream = None
            result = None
            duration_keys = ('pause_duration', 'settle_duration', 'measure_duration')
            metadata = dict(
                self.settings, pose_frames=self.frame, total_distance=self.total_distance,
                planned_segments=len(self.schedule), completed_segments=len(self.segments),
                elapsed_seconds=time.monotonic() - self.started,
                planned_seconds=len(self.schedule) * sum(self.settings[k] for k in duration_keys))
            try:
                if error is None:
                    result = fit_calibration(
                        self.segments, self.settings['max_fit_rmse'],
                        self.settings['max_direction_difference'])
                    write_result(self.directory, self.segments, result, metadata)
                    self.status('Identified coefficients: ' + json.dumps(result['coefficients']))
            except Exception as failure:
                error = str(failure)
            try:
                write_report(self.directory, self.segments, result, metadata, error)
                status = 'FAILED: ' + error if error else 'COMPLETED'
                self.status(f'{status}; report={self.directory / "report.html"}')
            except Exception as failure:
                self.status(f'Report error: {failure}; measurements={self.directory}')

    rclpy.init(args=arguments)
    node = None
    try:
        node = CalibrationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.active:
                node.finish('Node shutdown')
            if rclpy.ok():
                node.send()
            elif node.stream:
                node.stream.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
