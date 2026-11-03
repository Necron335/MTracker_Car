"""Validate pose identification with a known nonlinear steering law and noise."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    'calibration_identification',
    Path(__file__).resolve().parents[1] / 'scripts' / 'calibration_identification.py')
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)

COEFFICIENTS = np.array([0.04, 0.95, 0.12, 0.10])


def true_angle(raw, coefficients=COEFFICIENTS):
    """Invert a known monotonic cubic to produce synthetic measurements."""
    low, high = -1.2, 1.2
    for _ in range(60):
        middle = (low + high) / 2
        if np.polynomial.polynomial.polyval(middle, coefficients) < raw:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def poses(angle, direction=1, noise=False):
    """Generate exact bicycle arcs, wrapped headings and firmware quantization."""
    rng = np.random.default_rng(5)
    time = np.linspace(1, 9, 401)
    distance = direction * 0.05 * (time - time[0])
    curvature = np.tan(angle) / 0.19
    yaw0 = 3.0
    turn = distance * curvature
    length = distance * np.sinc(turn / (2*np.pi))
    x = length * np.cos(yaw0 + turn/2)
    y = length * np.sin(yaw0 + turn/2)
    yaw = yaw0 + turn
    if noise:
        x += rng.normal(0, 0.0001, len(x))
        y += rng.normal(0, 0.0001, len(y))
        yaw += rng.normal(0, 0.0002, len(yaw))
        x = np.round(x * 32768) / 32768
        y = np.round(y * 32768) / 32768
        yaw = np.round(yaw * 1024) / 1024
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    return np.column_stack((time, x, y, yaw))


@pytest.mark.parametrize('angle', [-0.6, -0.2, 0, 0.2, 0.6])
@pytest.mark.parametrize('direction', [-1, 1])
def test_curvature_recovers_signed_angle(angle, direction):
    """Reverse motion and heading wrapping retain the same physical steering."""
    result = calibration.estimate_segment(poses(angle, direction), 0.2, direction, 0.19)
    assert result['angle'] == pytest.approx(angle, abs=1e-9)
    assert result['distance'] == pytest.approx(0.4, abs=1e-9)


def test_identifies_cubic_from_54_noisy_runs():
    """Recover offset, slope and nonlinear terms from a substantial repeated sweep."""
    schedule = calibration.make_schedule(np.linspace(-0.6, 0.6, 9), repeats=3)
    assert len(schedule) == 54
    segments = [calibration.estimate_segment(
        poses(true_angle(item['raw']), item['direction'], noise=True),
        item['raw'], item['direction'], 0.19) for item in schedule]
    result = calibration.fit_calibration(segments)
    np.testing.assert_allclose(result['coefficients'], COEFFICIENTS, atol=0.003)
    assert result['rmse_raw'] < 0.001
    assert result['cross_validation_rmse_raw'] < 0.002


@pytest.mark.parametrize('fault', ['reverse', 'stopped', 'jump', 'stale', 'nan', 'yaw_jump'])
def test_rejects_bad_measurement_segments(fault):
    """Do not generate a steering estimate from invalid or unobservable motion."""
    sample = poses(0.2)
    if fault == 'reverse':
        sample = poses(0.2, -1)
    elif fault == 'stopped':
        sample[:, 1:] = 0
    elif fault == 'jump':
        sample[50:, 1] += 1
    elif fault == 'stale':
        sample[50:, 0] += 1
    elif fault == 'nan':
        sample[50, 1] = np.nan
    else:
        sample[50:, 3] += 1
    with pytest.raises(ValueError):
        calibration.estimate_segment(sample, 0.2, 1, 0.19)


def test_rejects_hysteresis_and_missing_direction():
    """A single-valued cubic must agree for forward and reverse motion."""
    segments = [{'raw': float(raw), 'angle': float(raw + (0.15 if direction == -1 else 0)),
                 'direction': direction}
                for raw in np.linspace(-0.6, 0.6, 9) for direction in (-1, 1)]
    with pytest.raises(ValueError, match='disagreement'):
        calibration.fit_calibration(segments)
    with pytest.raises(ValueError, match='forward/reverse'):
        calibration.fit_calibration([s for s in segments if s['direction'] == 1])


def test_negative_steering_direction_is_supported():
    """Firmware coefficient signs, rather than a fixed sign assumption, define steering."""
    segments = [{'raw': float(-angle), 'angle': float(angle), 'direction': direction}
                for angle in np.linspace(-0.6, 0.6, 9) for direction in (-1, 1)]
    result = calibration.fit_calibration(segments)
    np.testing.assert_allclose(result['coefficients'], [0, -1, 0, 0], atol=1e-10)


def test_schedule_pairs_directions_and_changes_history():
    """Every level is measured in both directions in each reproducible repetition."""
    levels = np.linspace(-0.6, 0.6, 9)
    steps = calibration.make_schedule(levels, 4)
    assert steps == calibration.make_schedule(levels, 4)
    for repeat in range(1, 5):
        block = [s for s in steps if s['repeat'] == repeat]
        assert {(s['raw'], s['direction']) for s in block} == {
            (raw, direction) for raw in levels for direction in (-1, 1)}
    assert steps[0]['raw'] != steps[18]['raw']


def test_schedule_full_servo_range():
    """Include both servo endpoints in both directions without clipping."""
    levels = [-1.0, -0.8, -0.6, -0.45, -0.3, -0.15, 0.0,
              0.15, 0.3, 0.45, 0.6, 0.8, 1.0]
    steps = calibration.make_schedule(levels, 3)
    assert len(steps) == 78
    for raw in (-1.0, 1.0):
        assert sum(s['raw'] == raw for s in steps) == 6
        assert {s['direction'] for s in steps if s['raw'] == raw} == {-1, 1}


@pytest.mark.parametrize('endpoint', [-1.001, 1.001])
def test_schedule_rejects_outside_servo_range(endpoint):
    """Reject commands exceeding the protocol's raw servo range."""
    with pytest.raises(ValueError, match=r'within \[-1,1\]'):
        calibration.make_schedule(sorted([endpoint, -0.5, 0.0, 0.5, 0.75]), 2)


def test_export_report(tmp_path):
    """Generate real standalone HTML, PDF and plots, also for incomplete acquisition."""
    import csv
    schedule = calibration.make_schedule(np.linspace(-0.6, 0.6, 9), 2)
    segments = []
    with (tmp_path / 'poses.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['step', 'repeat', 'raw', 'direction', 'stamp', 'x', 'y', 'theta'])
        for step, item in enumerate(schedule):
            samples = poses(true_angle(item['raw']), item['direction'])
            segments.append(calibration.estimate_segment(
                samples, item['raw'], item['direction'], 0.19))
            for pose in samples:
                writer.writerow([step, item['repeat'], item['raw'], item['direction'], *pose])
    result = calibration.fit_calibration(segments)
    calibration.write_result(tmp_path, segments, result, {'wheel_base': 0.19})
    calibration.write_report(tmp_path, segments, result, {'wheel_base': 0.19})
    assert 'data:image/png;base64,' in (tmp_path / 'report.html').read_text()
    assert (tmp_path / 'calibration.pdf').read_bytes().startswith(b'%PDF')
    assert (tmp_path / 'calibration.png').read_bytes().startswith(b'\x89PNG')
    assert (tmp_path / 'steering_calibration.yaml').exists()
    replay_segments, replay_result = calibration.identify_csv(tmp_path / 'poses.csv', 0.19)
    assert len(replay_segments) == len(segments)
    np.testing.assert_allclose(replay_result['coefficients'], result['coefficients'])
    calibration.write_report(tmp_path, [], None, {}, 'Cancelled by operator')
    assert 'Cancelled by operator' in (tmp_path / 'report.html').read_text()


@pytest.mark.parametrize('outcome', ['completed', 'pose_loss', 'driver_loss', 'cancel'])
def test_acquisition_state_machine_without_hardware(monkeypatch, tmp_path, outcome):
    """Drive the actual node callbacks with a simulated clock and nonlinear plant."""
    import sys
    from types import SimpleNamespace
    import rclpy
    import rclpy.node
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    try:
        import steering_calibration as acquisition
    finally:
        sys.path.pop(0)
    clock = [100.0]
    reports = []
    monkeypatch.setattr(acquisition.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(acquisition, 'write_report', lambda *args: reports.append(args))
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)

    class Publisher:
        def __init__(self):
            self.last = None

        def publish(self, message):
            self.last = message

        def get_subscription_count(self):
            return 1

    class FakeNode:
        def __init__(self, name):
            self.name = name

        def declare_parameter(self, name, default):
            overrides = {'enable_motion': True, 'output_dir': str(tmp_path), 'repeats': 2,
                         'raw_values': [-0.6, -0.3, 0.0, 0.3, 0.6]}
            return SimpleNamespace(value=overrides.get(name, default))

        def create_publisher(self, *args):
            return Publisher()

        def create_subscription(self, *args):
            return None

        def create_service(self, *args):
            return None

        def create_timer(self, *args):
            return None

        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9)))

        def destroy_node(self):
            pass

    monkeypatch.setattr(rclpy.node, 'Node', FakeNode)
    monkeypatch.setitem(
        sys.modules, 'mtrackercar.msg', SimpleNamespace(CarRawCommand=SimpleNamespace))
    monkeypatch.setitem(sys.modules, 'mtrackercar', SimpleNamespace())

    def experiment(node):
        x, y, yaw = 0.0, 0.0, 0.1

        def feedback():
            msg = Odometry()
            sec = int(clock[0])
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = int((clock[0] - sec) * 1e9)
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.w = np.cos(yaw / 2)
            msg.pose.pose.orientation.z = np.sin(yaw / 2)
            node.on_pose(msg)

        feedback()
        node.driver_ok, node.driver_received = True, clock[0]
        response = node.start(Trigger.Request(), Trigger.Response())
        assert response.success
        for index in range(6000):
            clock[0] += 0.05
            last = node.publisher.last
            speed = last.rear_velocity_rad_s * 0.033
            angle = true_angle(last.raw_servo)
            turn = speed * np.tan(angle) / 0.19 * 0.05
            length = speed * 0.05 * np.sinc(turn / (2*np.pi))
            x += length * np.cos(yaw + turn/2)
            y += length * np.sin(yaw + turn/2)
            yaw += turn
            if outcome != 'pose_loss' or index < 40:
                feedback()
            if outcome != 'driver_loss' or index < 40:
                node.driver_received = clock[0]
            if outcome == 'cancel' and index == 40:
                node.cancel(Trigger.Request(), Trigger.Response())
            node.update()
            if not node.active:
                break
        assert not node.active, 'Acquisition did not terminate'
        assert node.publisher.last.rear_velocity_rad_s == 0
        assert node.publisher.last.raw_servo == 0
        assert reports
        directory = node.directory
        if outcome == 'completed':
            assert len(node.segments) == 20
            import yaml
            result = yaml.safe_load((directory / 'identification.yaml').read_text())
            np.testing.assert_allclose(result['identification']['coefficients'],
                                       COEFFICIENTS, atol=0.003)
        else:
            assert not (directory / 'steering_calibration.yaml').exists()
            assert reports[-1][-1] is not None

    monkeypatch.setattr(rclpy, 'spin', experiment)
    acquisition.run_ros([])
