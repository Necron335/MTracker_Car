"""Identify raw-servo = cubic(physical steering) from measured planar poses."""

import csv
import json
from pathlib import Path

import numpy as np
import yaml


def robust_line(x, y):
    """Fit heading against signed travelled distance with Huber weights."""
    matrix = np.column_stack((np.ones(len(x)), x))
    weights = np.ones(len(x))
    for _ in range(12):
        root = np.sqrt(weights)
        fit, _, rank, _ = np.linalg.lstsq(matrix * root[:, None], y * root, rcond=None)
        if rank != 2:
            raise ValueError('Insufficient displacement to estimate curvature')
        residual = y - matrix @ fit
        scale = max(0.001, 1.4826 * np.median(np.abs(residual - np.median(residual))))
        weights = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1e-12))
    return float(fit[1]), float(np.sqrt(np.mean(residual ** 2)))


def estimate_segment(samples, raw, direction, wheel_base, min_distance=0.15,
                     max_pose_gap=0.3, max_yaw_rms=0.025):
    """Estimate bicycle steering from [timestamp, rear-axle x, y, yaw] samples."""
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 4 or len(values) < 10:
        raise ValueError('At least 10 fresh poses per segment are required')
    if not np.isfinite(values).all() or direction not in (-1, 1):
        raise ValueError('Invalid pose or direction')
    if not np.isfinite([raw, wheel_base, min_distance, max_pose_gap, max_yaw_rms]).all():
        raise ValueError('Nonfinite identification settings')
    if abs(raw) > 1 or min(wheel_base, min_distance, max_pose_gap, max_yaw_rms) <= 0:
        raise ValueError('Invalid identification settings')
    dt = np.diff(values[:, 0])
    if np.any(dt <= 0) or np.max(dt) > max_pose_gap:
        raise ValueError('Nonmonotonic timestamps or missing poses')
    heading = np.unwrap(values[:, 3])
    turns = np.diff(heading)
    if np.max(np.abs(turns)) > 0.3:
        raise ValueError('Orientation discontinuity')
    middle = heading[:-1] + turns / 2
    delta = np.diff(values[:, 1:3], axis=0)
    longitudinal = delta[:, 0] * np.cos(middle) + delta[:, 1] * np.sin(middle)
    lateral = -delta[:, 0] * np.sin(middle) + delta[:, 1] * np.cos(middle)
    # Project chords onto midpoint heading and recover signed arc lengths.
    ds = longitudinal / np.sinc(turns / (2 * np.pi))
    if np.any(np.linalg.norm(delta, axis=1) > 0.30 * dt + 0.02):
        raise ValueError('Pose jump or excessive measured speed')
    distance = np.concatenate(([0.0], np.cumsum(ds)))
    travel = float(direction * distance[-1])
    if travel < min_distance:
        raise ValueError('Insufficient travel or motion opposite to commanded direction')
    if np.sum(np.maximum(-direction * ds - 0.002, 0)) > 0.05 * travel:
        raise ValueError('Inconsistent direction during measurement')
    if np.sqrt(np.mean(lateral ** 2)) > 0.01:
        raise ValueError('Excessive lateral motion; check pose frame and rear-axle offset')
    curvature, yaw_rms = robust_line(distance, heading - heading[0])
    if yaw_rms > max_yaw_rms:
        raise ValueError('Curvature is not constant; steering may not have settled')
    angle = float(np.arctan(wheel_base * curvature))
    return {
        'raw': float(raw), 'direction': int(direction), 'angle': angle,
        'curvature': curvature, 'distance': travel, 'yaw_rms': yaw_rms,
        'samples': len(values), 'duration': float(values[-1, 0] - values[0, 0])}


def fit_calibration(segments, max_rmse=0.04, max_direction_difference=0.06,
                    min_angle_span=0.20):
    """Fit the firmware polynomial and reject unobservable or hysteretic results."""
    if len(segments) < 8:
        raise ValueError('At least four servo levels in both directions are required')
    groups = {}
    for item in segments:
        groups.setdefault(item['raw'], []).append(item)
    if len(groups) < 4:
        raise ValueError('At least four distinct servo levels are required')
    asymmetry = 0.0
    for raw, items in groups.items():
        means = []
        for direction in (1, -1):
            angles = [v['angle'] for v in items if v['direction'] == direction]
            if not angles:
                raise ValueError(f'Missing forward/reverse observation for raw={raw}')
            means.append(float(np.mean(angles)))
        asymmetry = max(asymmetry, abs(means[0] - means[1]))
    if asymmetry > max_direction_difference:
        raise ValueError('Forward/reverse steering disagreement: backlash, slip or bad geometry')
    angle = np.array([item['angle'] for item in segments])
    raw = np.array([item['raw'] for item in segments])
    if not np.isfinite(angle).all() or not np.isfinite(raw).all():
        raise ValueError('Nonfinite segment estimates')
    if float(np.ptp(angle)) < min_angle_span:
        raise ValueError('Insufficient steering angle span')
    if angle.min() >= 0 or angle.max() <= 0:
        raise ValueError('Measurements must bracket straight steering')
    # Center/scale the Vandermonde matrix before QR/SVD least squares.
    center = float(np.mean(angle))
    scale = float(np.max(np.abs(angle - center)))
    z = (angle - center) / scale
    matrix = np.polynomial.polynomial.polyvander(z, 3)
    coefficients, _, rank, _ = np.linalg.lstsq(matrix, raw, rcond=None)
    condition = float(np.linalg.cond(matrix))
    if rank != 4 or condition > 1000:
        raise ValueError('Steering observations do not identify a cubic reliably')
    polynomial = np.polynomial.Polynomial(coefficients)(
        np.polynomial.Polynomial([-center / scale, 1 / scale]))
    c = np.pad(polynomial.coef, (0, max(0, 4 - len(polynomial.coef))))[:4]
    residuals = np.polynomial.polynomial.polyval(angle, c) - raw
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    # Leave one servo level out: reject a cubic that only interpolates the sweep.
    cv_errors = []
    if len(groups) >= 5:
        for level in groups:
            train = raw != level
            candidate, _, cv_rank, _ = np.linalg.lstsq(matrix[train], raw[train], rcond=None)
            if cv_rank != 4:
                raise ValueError('Unstable leave-one-level-out fit')
            cv_errors.extend(matrix[~train] @ candidate - raw[~train])
    cv_rmse = float(np.sqrt(np.mean(np.square(cv_errors)))) if cv_errors else None
    if rmse > max_rmse or (cv_rmse is not None and cv_rmse > 2 * max_rmse):
        raise ValueError('Cubic prediction error exceeds the acceptance threshold')
    if abs(c[0]) > 1 or np.max(np.abs(c[1:])) > 1000:
        raise ValueError('Coefficients exceed firmware limits')
    grid = np.linspace(angle.min(), angle.max(), 200)
    derivative = c[1] + 2 * c[2] * grid + 3 * c[3] * grid ** 2
    if not (np.all(derivative > 0) or np.all(derivative < 0)):
        raise ValueError('Fitted steering map is not monotonic inside the measured range')
    return {
        'coefficients': [float(v) for v in c], 'rmse_raw': rmse,
        'cross_validation_rmse_raw': cv_rmse, 'condition_number': condition,
        'forward_reverse_difference_rad': asymmetry,
        'angle_range_rad': [float(angle.min()), float(angle.max())],
        'raw_range': [float(raw.min()), float(raw.max())], 'segments': len(segments)}


def identify_csv(filename, wheel_base, **kwargs):
    """Repeat identification offline from the recorded measurement rows."""
    groups = {}
    with open(filename, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            key = (int(row['step']), float(row['raw']), int(row['direction']))
            groups.setdefault(key, []).append([
                float(row[name]) for name in ('stamp', 'x', 'y', 'theta')])
    segments = [estimate_segment(values, raw, direction, wheel_base, **kwargs)
                for (_, raw, direction), values in groups.items()]
    return segments, fit_calibration(segments)


def write_result(directory, segments, result, metadata):
    """Export fit diagnostics and a ROS parameter file without applying calibration."""
    directory = Path(directory)
    report = {'metadata': metadata, 'identification': result, 'measurements': segments}
    (directory / 'identification.yaml').write_text(
        yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
    parameters = {'mtrackercar': {'ros__parameters': {
        'steering_calibration': result['coefficients']}}}
    (directory / 'steering_calibration.yaml').write_text(
        yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8')
    return json.dumps(result)


def make_schedule(raw_values, repeats=3, seed=7):
    """Pair forward/reverse runs and vary level order to reveal history effects."""
    values = np.asarray(raw_values, dtype=float)
    if (len(values) < 5 or len(set(values)) != len(values) or
            not np.isfinite(values).all() or np.max(np.abs(values)) > 1.0 or
            values.min() >= 0 or values.max() <= 0 or repeats < 2):
        raise ValueError(
            'Use >=5 distinct servo levels spanning zero within [-1,1], >=2 repeats')
    random = np.random.default_rng(seed)
    steps = []
    for repeat in range(repeats):
        order = sorted(values) if repeat == 0 else sorted(values, reverse=True)
        if repeat >= 2:
            random.shuffle(order)
        directions = (1, -1) if repeat % 2 == 0 else (-1, 1)
        for raw in order:
            for direction in directions:
                steps.append({'raw': float(raw), 'direction': direction, 'repeat': repeat + 1})
    return steps


def write_report(directory, segments, result, metadata, error=None):
    """Render an exportable figure, PDF and self-contained HTML report."""
    import base64
    import html
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    directory = Path(directory)
    rows = []
    csv_path = directory / 'poses.csv'
    if csv_path.exists():
        with csv_path.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    trajectory, mapping, residual_plot, repeatability = axes.flat
    if rows:
        steps = {}
        for row in rows:
            steps.setdefault(row['step'], []).append(row)
        for points in steps.values():
            raw = float(points[0]['raw'])
            color = plt.cm.coolwarm((raw + 1.0) / 2.0)
            trajectory.plot([float(p['x']) for p in points],
                            [float(p['y']) for p in points], color=color, alpha=0.65)
        trajectory.scatter(float(rows[0]['x']), float(rows[0]['y']),
                           marker='*', color='black', label='Start')
        trajectory.legend()
    trajectory.set(title='Measured trajectories (colour: raw servo)',
                   xlabel='x [m]', ylabel='y [m]')
    trajectory.set_aspect('equal', adjustable='datalim')
    c = result['coefficients'] if result else None
    for direction, label, marker, color in [(1, 'Forward', 'o', 'tab:blue'),
                                            (-1, 'Reverse', 'x', 'tab:orange')]:
        items = [s for s in segments if s['direction'] == direction]
        if not items:
            continue
        angles = np.array([s['angle'] for s in items])
        raw = np.array([s['raw'] for s in items])
        mapping.scatter(np.degrees(angles), raw, label=label, marker=marker, color=color)
        if c is not None:
            residual_plot.scatter(np.degrees(angles),
                                  np.polynomial.polynomial.polyval(angles, c) - raw,
                                  label=label, marker=marker, color=color)
        levels = sorted(set(raw))
        means = [np.mean([s['angle'] for s in items if s['raw'] == r]) for r in levels]
        std = [np.std([s['angle'] for s in items if s['raw'] == r]) for r in levels]
        repeatability.errorbar(levels, np.degrees(means), yerr=np.degrees(std),
                               marker=marker, color=color, label=label, capsize=3)
    if c is not None:
        a = np.linspace(*result['angle_range_rad'], 200)
        mapping.plot(np.degrees(a), np.polynomial.polynomial.polyval(a, c),
                     color='black', label='Identified cubic')
    mapping.set(title='Calibration: rawServo = polynomial(angle)',
                xlabel='Measured steering angle [deg]', ylabel='rawServo [-]')
    residual_plot.axhline(0, color='black', linewidth=0.8)
    residual_plot.set(title='Polynomial residuals', xlabel='Measured steering angle [deg]',
                      ylabel='Predicted raw - commanded raw [-]')
    repeatability.set(title='Direction agreement and repeatability (mean ± std)',
                      xlabel='Commanded rawServo [-]', ylabel='Measured steering angle [deg]')
    for ax in axes.flat:
        ax.grid(alpha=0.3)
        handles, _ = ax.get_legend_handles_labels()
        if handles:
            ax.legend()
    demonstration = 'SYNTHETIC' in str(metadata.get('data', '')).upper()
    suffix = ' — SYNTHETIC DATA' if demonstration else ''
    fig.suptitle('MTracker steering identification' + suffix + (' — FAILED' if error else ''))
    fig.savefig(directory / 'calibration.png', dpi=160)
    fig.savefig(directory / 'calibration.pdf')
    plt.close(fig)
    if segments:
        with (directory / 'segments.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(segments[0]))
            writer.writeheader()
            writer.writerows(segments)
    payload = {'metadata': metadata, 'result': result, 'error': error,
               'completed_segments': len(segments)}
    (directory / 'report.yaml').write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    image = base64.b64encode((directory / 'calibration.png').read_bytes()).decode('ascii')
    summary = html.escape(yaml.safe_dump(payload, sort_keys=False))
    status = (html.escape(error) if error else
              'Identification completed; coefficients have not been applied.')
    document = f"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>MTracker steering identification</title><style>
body {{font:16px sans-serif;max-width:1100px;margin:2em auto;padding:0 1em;}}
img {{width:100%;}} pre {{background:#f3f4f6;padding:1em;overflow:auto;}}
</style><h1>MTracker steering identification</h1><p>{status}</p>
<img alt="Trajectories, cubic calibration, residuals and direction repeatability"
src="data:image/png;base64,{image}"><h2>Identification and acquisition settings</h2>
<pre>{summary}</pre><p>Angles are inferred from planar pose using the rear-axle bicycle model.
Wheel slip and incorrect odometry geometry bias the result. Use coefficients only within the
measured angle range. The EEPROM retention test requires the physical vehicle.</p></html>"""
    (directory / 'report.html').write_text(document, encoding='utf-8')
