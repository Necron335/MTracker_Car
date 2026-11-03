"""Velocity experiments and bounded, validated tuning of the CAR left controller."""

import csv
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import numpy as np
import yaml

PARAMETER_NAMES = ('feedback_bandwidth', 'observer_bandwidth', 'damping', 'inertia')
FIRMWARE_DEFAULTS = (15.0, 80.0, 1.0, 0.0025)
COLUMNS = ('index', 'time_s', 'reference_velocity_m_s', 'velocity_m_s',
           'left_position_error_rad', 'left_output')
TELEMETRY_COLUMNS = ('index', 'time_s', 'requested_velocity_m_s', 'velocity_m_s',
                     'motor_output')


class TrialQualityRejected(ValueError):
    """A complete capture fails quality checks but may permit another trial."""


def evaluate_telemetry(rows, profile, sample_period, wheel_radius=0.033):
    """Score CMD=5 velocity tracking; no internal controller error is inferred."""
    del wheel_radius
    data = np.asarray(rows, dtype=float)
    if (data.ndim != 2 or data.shape[1] != 5 or len(data) < 20 or
            not np.isfinite(data).all() or not math.isfinite(sample_period) or
            sample_period <= 0):
        raise ValueError('Invalid or insufficient CMD=5 telemetry')
    index, t, reference, measured, output = data.T
    dt = np.diff(t)
    if (index[0] != 0 or not np.all(np.diff(index) == 1) or np.any(dt <= 0) or
            np.max(dt) > 0.15 or np.median(dt) > 0.05):
        raise ValueError('Telemetry gaps or sample rate below 20 Hz')
    if t[0] < 0 or t[0] > 0.15 or t[-1] < profile.duration-0.15:
        raise ValueError('Incomplete telemetry duration')
    if t[-1] > profile.duration+0.15 or np.max(np.abs(reference)) > profile.amplitude+0.005:
        raise ValueError('Telemetry exceeds requested duration or reference amplitude')
    expected = np.array([profile.value(x) for x in t])
    if np.mean(np.abs(reference-expected)) > profile.amplitude*0.20:
        raise ValueError('Acknowledged requests do not match the experiment')
    if np.max(np.abs(output)) > 1.01 or np.max(np.abs(measured)) > 0.15:
        raise ValueError('Excessive measured speed or invalid motor output')
    if np.dot(reference, measured) <= 0:
        raise ValueError('No response or motion opposite to requested direction')
    saturation = float(np.mean(np.abs(output) >= 0.98))
    if saturation > 0.30:
        raise ValueError('Excessive motor-output saturation')
    quiet = t >= profile.duration-profile.quiet/2
    if not np.any(quiet) or np.max(np.abs(measured[quiet])) > max(0.015, profile.amplitude*0.20):
        raise TrialQualityRejected('Robot did not settle after the final stop')
    tracking = reference-measured
    duration = t[-1]-t[0]
    rmse = float(np.sqrt(np.trapz(tracking**2, t)/duration))
    activity = float(np.sum(np.diff(output)**2)/duration)
    return {'score': rmse/profile.amplitude + 0.02*activity + 0.5*saturation,
            'tracking_rmse_m_s': rmse,
            'integral_absolute_error_m': float(np.trapz(np.abs(tracking), t)),
            'output_rms': float(np.sqrt(np.trapz(output**2, t)/duration)),
            'output_activity': activity, 'saturation_fraction': saturation,
            'peak_speed_m_s': float(np.max(np.abs(measured)))}


@dataclass(frozen=True)
class Profile:
    """A finite velocity reference with zero lead-in and lead-out intervals."""

    kind: str = 'steps'
    amplitude: float = 0.05
    duration: float = 12.0
    quiet: float = 1.0
    frequency_start: float = 0.10
    frequency_end: float = 1.0
    seed: int = 7

    def validate(self, max_speed=0.10):
        """Reject profiles with invalid duration, amplitude or frequency."""
        if self.kind not in ('steps', 'ramp', 'sine', 'chirp', 'prbs'):
            raise ValueError('Unknown velocity profile')
        values = (self.amplitude, self.duration, self.quiet,
                  self.frequency_start, self.frequency_end, max_speed)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Profile settings must be finite and positive')
        if self.amplitude > max_speed or max_speed > 0.10 or self.duration <= 2*self.quiet:
            raise ValueError('Speed exceeds 0.10 m/s or profile has no active interval')
        if self.frequency_end < self.frequency_start:
            raise ValueError('End frequency must be >= start frequency')
        return self

    def value(self, elapsed):
        """Evaluate a bounded reference in physical m/s; reverse is negative."""
        t = elapsed - self.quiet
        active = self.duration - 2*self.quiet
        if t < 0 or t >= active:
            return 0.0
        u = t / active
        if self.kind == 'steps':
            levels = (0.5, 1.0, 0.0, -0.5, -1.0, 0.0)
            value = levels[min(5, int(6*u))]
        elif self.kind == 'ramp':
            value = np.interp(u, [0, 0.25, 0.5, 0.75, 1], [0, 1, 0, -1, 0])
        elif self.kind == 'sine':
            value = math.sin(2*math.pi*self.frequency_start*t)
        elif self.kind == 'chirp':
            rate = (self.frequency_end - self.frequency_start) / active
            value = math.sin(2*math.pi*(self.frequency_start*t + 0.5*rate*t*t))
        else:
            # A fixed, seeded sequence is replayed identically for each candidate.
            random = np.random.default_rng(self.seed)
            levels = random.choice([-1.0, 1.0], 256)
            value = levels[int(t*self.frequency_end) % len(levels)]
        return float(self.amplitude * value)


def validate_samples(rows, profile, sample_period, overflow=False, max_gap_factor=2.5):
    """Verify an entire firmware capture before evaluating a candidate."""
    if overflow or not math.isfinite(sample_period) or sample_period <= 0:
        raise ValueError('Firmware buffer overflow or invalid sample period')
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or data.shape[1] != len(COLUMNS) or len(data) < 20:
        raise ValueError('Insufficient firmware buffer samples')
    if not np.isfinite(data).all():
        raise ValueError('Nonfinite buffer sample')
    if data[0, 0] != 0 or not np.array_equal(np.diff(data[:, 0]), np.ones(len(data)-1)):
        raise ValueError('Missing, duplicate or reordered firmware sample indices')
    dt = np.diff(data[:, 1])
    if (np.any(dt < sample_period*0.4) or np.any(dt > sample_period*max_gap_factor) or
            abs(float(np.mean(dt))/sample_period - 1) > 0.05):
        raise ValueError('Discontinuous firmware sample timestamps')
    if data[0, 1] > 2*sample_period or data[0, 1] < 0:
        raise ValueError('Capture does not begin at the firmware trigger')
    if data[-1, 1] < profile.duration - 2*sample_period:
        raise ValueError('Incomplete capture duration')
    if data[-1, 1] > profile.duration + 1.0:
        raise ValueError('Capture duration does not match experiment')
    if np.max(np.abs(data[:, 2])) > profile.amplitude + 0.005:
        raise ValueError('Firmware reference exceeds requested profile amplitude')
    expected = np.array([profile.value(t) for t in data[:, 1]])
    # Ignore a small window at commanded discontinuities; the applied reference
    # remains recorded from the buffer, not reconstructed from this array.
    jump = np.flatnonzero(np.abs(np.diff(expected)) > profile.amplitude*0.20)
    usable = np.ones(len(data), dtype=bool)
    for index in jump:
        usable[max(0, index-3):index+5] = False
    if np.mean(np.abs(data[usable, 2] - expected[usable])) > profile.amplitude*0.15:
        raise ValueError('Applied firmware reference does not match requested experiment')
    if np.max(np.abs(data[:, 5])) > 1.01:
        raise ValueError('Left controller output is not normalized to [-1,1]')
    return data


def evaluate(rows, profile, sample_period, wheel_radius=0.033,
             max_speed=0.15, max_saturation_fraction=0.30):
    """Compute a tracking/control-effort objective with hard feasibility limits."""
    data = validate_samples(rows, profile, sample_period)
    t, reference, measured, error, output = data[:, 1:].T
    if not math.isfinite(wheel_radius) or wheel_radius <= 0:
        raise ValueError('Invalid wheel radius')
    if np.max(np.abs(measured)) > max_speed:
        raise ValueError('Measured robot speed exceeds the experiment limit')
    saturation = float(np.mean(np.abs(output) >= 0.98))
    if saturation > max_saturation_fraction:
        raise ValueError('Excessive left-output saturation')
    quiet = t >= profile.duration - profile.quiet/2
    if np.any(quiet) and np.max(np.abs(measured[quiet])) > max(0.015, profile.amplitude*0.20):
        raise TrialQualityRejected('Robot did not settle after the final stop')
    tracking = reference - measured
    tracking_rmse = float(np.sqrt(np.mean(tracking**2)))
    integral_error = float(np.trapz(np.abs(tracking), t))
    left_error_rmse = float(np.sqrt(np.mean(error**2)))
    activity = float(np.mean(np.diff(output)**2) / sample_period)
    normalized_tracking = tracking_rmse / profile.amplitude
    # Normalize position-error length by travel at reference amplitude over 1 s.
    normalized_left_error = left_error_rmse * wheel_radius / (profile.amplitude * 1.0)
    score = normalized_tracking + 0.15*normalized_left_error + 0.02*activity + 0.5*saturation
    return {'score': float(score), 'tracking_rmse_m_s': tracking_rmse,
            'integral_absolute_error_m': integral_error,
            'left_position_error_rmse_rad': left_error_rmse,
            'output_rms': float(np.sqrt(np.mean(output**2))), 'output_activity': activity,
            'saturation_fraction': saturation, 'peak_speed_m_s': float(np.max(np.abs(measured)))}


def write_capture(path, rows, columns=COLUMNS):
    """Preserve original firmware sample values with explicit units and indices."""
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def read_capture(path):
    """Import the same CSV schema for offline scoring/reporting."""
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames
        if tuple(columns or ()) not in (COLUMNS, TELEMETRY_COLUMNS):
            raise ValueError('Unknown capture CSV schema')
        return [[float(row[name]) for name in columns] for row in reader]


def validate_parameters(parameters, bounds):
    """Constrain all four positive firmware parameters before a hardware write."""
    values = np.asarray(parameters, dtype=float)
    limits = np.asarray(bounds, dtype=float)
    if values.shape != (4,) or limits.shape != (4, 2):
        raise ValueError('Four parameter values and four [low,high] bounds are required')
    if not np.isfinite(values).all() or not np.isfinite(limits).all():
        raise ValueError('Nonfinite regulator parameters or bounds')
    if np.any(limits[:, 0] <= 0) or np.any(limits[:, 1] <= limits[:, 0]):
        raise ValueError('Invalid regulator bounds')
    firmware_low = np.array([1.0, 1.0, 0.1, 0.00001])
    firmware_high = np.array([200.0, 1000.0, 5.0, 1.0])
    if np.any(limits[:, 0] < firmware_low) or np.any(limits[:, 1] > firmware_high):
        raise ValueError('Search bounds exceed firmware parameter ranges')
    if values[1] < values[0] or limits[1, 0] < limits[0, 1]:
        raise ValueError('ESO bandwidth must remain >= feedback bandwidth throughout the search')
    if np.any(values < limits[:, 0]) or np.any(values > limits[:, 1]):
        raise ValueError('Regulator parameters are outside the allowed search region')
    return values


class TuneRunner:
    """Bounded coordinate pattern search with holdout validation and rollback.

    Backend methods: set_parameters(values), capture(profile), stop(), save().
    capture returns {rows, sample_period_s, overflow}. A telemetry backend must
    explicitly declare its own columns, acquisition label and scoring function.
    Buffer data is never reconstructed from host telemetry.
    """

    def __init__(self, backend, output_dir, baseline, bounds, budget=12,
                 wheel_radius=0.033, validation_repeats=2, min_improvement=0.03):
        self.backend = backend
        self.directory = Path(output_dir)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.bounds = np.asarray(bounds, dtype=float)
        self.baseline = validate_parameters(baseline, bounds)
        if budget < 1 or validation_repeats < 2 or not 0 <= min_improvement < 1:
            raise ValueError('Invalid optimization/validation budget')
        self.budget = budget
        self.wheel_radius = wheel_radius
        self.validation_repeats = validation_repeats
        self.min_improvement = min_improvement
        self.trials = []
        self.validated = False
        self.selected = self.baseline.copy()

    def trial(self, parameters, profile, phase):
        """Skip rejected quality samples; retry a validation measurement once."""
        attempts = 2 if phase.startswith('validation_') else 1
        for attempt in range(attempts):
            try:
                return self._trial_once(parameters, profile, phase)
            except TrialQualityRejected:
                # A historical settling failure is not evidence of current motion.
                # Confirm a real stop before proceeding to any further experiment.
                if hasattr(self.backend, 'wait_stopped'):
                    self.backend.wait_stopped()
                if attempt+1 == attempts:
                    return math.inf

    def _trial_once(self, parameters, profile, phase):
        """Stop, apply one candidate, capture all samples, and persist the trial."""
        values = validate_parameters(parameters, self.bounds)
        trial_id = len(self.trials)
        record = {'trial': trial_id, 'phase': phase, 'parameters': values.tolist(),
                  'profile': asdict(profile), 'accepted': False}
        try:
            if hasattr(self.backend, 'check_feedback'):
                self.backend.check_feedback()
            self.backend.stop()
            self.backend.set_parameters(values.tolist())
            capture = self.backend.capture(profile)
            self.backend.stop()
            record['capture'] = f'trial_{trial_id:03d}.csv'
            write_capture(self.directory / record['capture'], capture['rows'],
                          getattr(self.backend, 'columns', COLUMNS))
            record['sample_period_s'] = float(capture['sample_period_s'])
            if capture.get('overflow', False):
                raise ValueError('Firmware buffer overflow')
            scorer = getattr(self.backend, 'evaluate', evaluate)
            record['metrics'] = scorer(capture['rows'], profile, capture['sample_period_s'],
                                       self.wheel_radius)
            record['accepted'] = True
        except Exception as error:
            record['error'] = str(error)
            record['quality_rejected'] = isinstance(error, TrialQualityRejected)
            self.backend.stop()
        self.trials.append(record)
        self.checkpoint()
        if hasattr(self.backend, 'progress'):
            self.backend.progress(record)
        # A failed capture can mean loss of the robot link or an unsafe response;
        # abort the search instead of blindly sending another motion trial.
        if not record['accepted']:
            if record.get('quality_rejected'):
                raise TrialQualityRejected(record['error'])
            raise RuntimeError(f'Trial {trial_id} failed: {record["error"]}')
        return record['metrics']['score']

    def checkpoint(self, result=None):
        """Keep a machine-readable audit after each candidate, including failures."""
        document = {'method': 'bounded coordinate search in log parameter space',
                    'acquisition': getattr(self.backend, 'acquisition', 'firmware_buffer'),
                    'synthetic': getattr(self.backend, 'synthetic', False),
                    'baseline': self.baseline.tolist(), 'bounds': self.bounds.tolist(),
                    'trials': self.trials, 'result': result}
        (self.directory / 'tuning.yaml').write_text(
            yaml.safe_dump(document, sort_keys=False), encoding='utf-8')

    def run(self, training, validation, apply=False, persist=False):
        """Select the best observed candidate, independently validate, then commit."""
        training.validate()
        validation.validate()
        if training == validation:
            raise ValueError('Validation must use a different reference profile')
        if persist and not apply:
            raise ValueError('EEPROM persistence requires apply=True')
        result = {'validated': False, 'applied': False, 'persisted': False,
                  'best_parameters': dict(zip(PARAMETER_NAMES, self.baseline.tolist()))}
        try:
            best_score = self.trial(self.baseline, training, 'baseline_training')
            best = self.baseline.copy()
            log_best = np.log(best)
            log_limits = np.log(self.bounds)
            step = 0.20 * (log_limits[:, 1] - log_limits[:, 0])
            count = 1
            while count < self.budget and np.max(step) > 0.01:
                improved = False
                for axis in range(4):
                    for direction in (-1, 1):
                        if count >= self.budget:
                            break
                        candidate = log_best.copy()
                        candidate[axis] = np.clip(candidate[axis] + direction*step[axis],
                                                  *log_limits[axis])
                        if np.allclose(candidate, log_best):
                            continue
                        parameters = np.clip(
                            np.exp(candidate), self.bounds[:, 0], self.bounds[:, 1])
                        score = self.trial(parameters, training, 'search')
                        count += 1
                        if score < best_score:
                            best_score, best, log_best = score, parameters, candidate
                            improved = True
                if not improved:
                    step *= 0.5
            baseline_scores, best_scores = [], []
            for repeat in range(self.validation_repeats):
                # Alternate the order to reduce battery/temperature order bias.
                order = ('baseline', 'best') if repeat % 2 == 0 else ('best', 'baseline')
                for role in order:
                    parameters = self.baseline if role == 'baseline' else best
                    score = self.trial(parameters, validation, 'validation_' + role)
                    (baseline_scores if role == 'baseline' else best_scores).append(score)
            baseline_mean, best_mean = float(np.mean(baseline_scores)), float(np.mean(best_scores))
            valid_scores = all(math.isfinite(x) for x in baseline_scores+best_scores)
            improvement = ((baseline_mean - best_mean) / max(baseline_mean, 1e-12)
                           if valid_scores else None)
            # Require both mean improvement and no worse paired validation run.
            self.validated = (math.isfinite(best_score) and valid_scores and
                              improvement >= self.min_improvement and
                              all(b <= a for a, b in zip(baseline_scores, best_scores)))
            self.selected = best.copy() if self.validated else self.baseline.copy()
            result.update(validated=self.validated, best_training_score=float(best_score),
                          validation_baseline_scores=baseline_scores,
                          validation_best_scores=best_scores, relative_improvement=improvement,
                          best_parameters=dict(zip(PARAMETER_NAMES, best.tolist())),
                          selected_parameters=dict(zip(PARAMETER_NAMES, self.selected.tolist())))
            self.backend.stop()
            if apply and self.validated:
                if hasattr(self.backend, 'check_feedback'):
                    self.backend.check_feedback()
                self.backend.set_parameters(self.selected.tolist())
                result['applied'] = True
                if persist:
                    # The backend must confirm the CMD=8 completion, not just its ACK.
                    result['persist_attempted'] = True
                    result['persistence_outcome'] = 'unknown'
                    self.backend.save()
                    result['persisted'] = True
                    result['persistence_outcome'] = 'confirmed'
            else:
                self.backend.set_parameters(self.baseline.tolist())
            self.checkpoint(result)
            return result
        except Exception as error:
            result['error'] = str(error)
            try:
                self.backend.stop()
                restore = getattr(self.backend, 'restore_parameters', self.backend.set_parameters)
                restore(self.baseline.tolist())
                result['baseline_restored'] = True
                result['applied'] = False
                result['active_parameters'] = dict(zip(PARAMETER_NAMES, self.baseline.tolist()))
            except Exception as restore_error:
                result['baseline_restored'] = False
                result['restore_error'] = str(restore_error)
            self.checkpoint(result)
            raise
        finally:
            self.backend.stop()


def write_report(directory):
    """Plot captured signals and rank feasible candidates in a standalone report."""
    import base64
    import html
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    directory = Path(directory)
    document = yaml.safe_load((directory / 'tuning.yaml').read_text(encoding='utf-8'))
    trials = document['trials']
    telemetry = document.get('acquisition') == 'cmd5_telemetry'
    result = document.get('result') or {}
    accepted = [t for t in trials if t['accepted']]
    candidates = [t for t in accepted if t['phase'] in ('baseline_training', 'search')]
    best = min(candidates, key=lambda t: t['metrics']['score']) if candidates else None
    baseline = next((t for t in trials if t['phase'] == 'baseline_training'), None)
    displayed = [t for t in (baseline, best) if t is not None]
    if not displayed and trials:
        displayed = trials[-1:]
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), constrained_layout=True)
    for trial in displayed:
        if 'capture' not in trial:
            continue
        data = np.asarray(read_capture(directory / trial['capture']))
        if len(data) == 0:
            continue
        t = data[:, 1]
        label = f'Trial {trial["trial"]} ({trial["phase"]})'
        axes[0].plot(t, data[:, 2], '--', alpha=0.6, label=label + ' reference')
        axes[0].plot(t, data[:, 3], label=label + ' measured')
        axes[1].plot(t, data[:, 2]-data[:, 3] if telemetry else data[:, 4], label=label)
        axes[2].plot(t, data[:, 4] if telemetry else data[:, 5], label=label)
    axes[0].set(ylabel='Robot velocity [m/s]', title='Reference and buffered measured velocity')
    axes[1].set(ylabel='Left position error [rad]',
                title='Left controller position error from firmware buffer')
    axes[2].set(ylabel='Left output [-]', title='Left controller output from firmware buffer')
    if telemetry:
        axes[0].set_title('CMD=5 request and measured velocity (host receipt time)')
        axes[1].set(ylabel='Velocity error [m/s]', title='Request minus measured velocity')
        axes[2].set(ylabel='Motor output [-]', title='Motor output from CMD=5 telemetry')
    axes[2].axhline(1, color='red', linestyle=':', linewidth=0.8)
    axes[2].axhline(-1, color='red', linestyle=':', linewidth=0.8)
    for trial in accepted:
        marker = 'o' if trial['phase'] in ('baseline_training', 'search') else 'x'
        axes[3].scatter(trial['trial'], trial['metrics']['score'], marker=marker)
    axes[3].set(ylabel='Objective (lower is better)', xlabel='Trial number',
                title='Feasible trials: circles = training; crosses = holdout validation')
    for ax in axes:
        ax.grid(alpha=0.3)
        handles, _ = ax.get_legend_handles_labels()
        if handles:
            ax.legend(fontsize=8)
    for ax in axes[:3]:
        ax.set_xlabel('Host elapsed time [s]' if telemetry else 'Firmware time from trigger [s]')
    synthetic = document.get('synthetic', False)
    title = ('MTracker CMD=5 velocity tuning' if telemetry
             else 'MTracker left-controller experiments')
    fig.suptitle(title + (' — SYNTHETIC DATA' if synthetic else ''))
    fig.savefig(directory / 'regulator.png', dpi=150)
    fig.savefig(directory / 'regulator.pdf')
    plt.close(fig)
    image = base64.b64encode((directory / 'regulator.png').read_bytes()).decode('ascii')
    ranked = sorted(candidates or accepted, key=lambda t: t['metrics']['score'])
    failures = html.escape(yaml.safe_dump([t for t in trials if not t['accepted']]))
    rows = ''.join(
        '<tr><td>' + str(t['trial']) + '</td><td>' + html.escape(t['phase']) +
        '</td><td>' + f'{t["metrics"]["score"]:.5f}' + '</td><td>' +
        (html.escape(', '.join(f'{v:.6g}' for v in t['parameters']))
         if t['parameters'] else 'Not changed') + '</td></tr>'
        for t in ranked)
    heading = title + (' — SYNTHETIC DATA' if synthetic else '')
    markup = f"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>{heading}</title><style>body{{font:16px sans-serif;max-width:1100px;margin:2em auto;
padding:0 1em}}img{{width:100%}}td,th{{border:1px solid #ccc;padding:.5em}}
table{{border-collapse:collapse}}pre{{background:#f3f4f6;padding:1em;overflow:auto}}</style>
<h1>{heading}</h1>
<p>Acquisition: {html.escape(document.get('acquisition', 'firmware_buffer'))}.</p>
<p>Best evaluated parameters within configured bounds;
this search does not prove a global optimum. Right controller is unused.</p>
<img alt="Velocity, left error, left output and trial scores" src="data:image/png;base64,{image}">
<h2>Ranking</h2><table><tr><th>Trial</th><th>Phase</th><th>Score</th>
<th>Feedback bandwidth, ESO bandwidth, damping, inertia</th></tr>{rows}</table>
<h2>Validation, application and EEPROM status</h2><pre>{html.escape(yaml.safe_dump(result))}</pre>
<h2>Failed trials</h2><pre>{failures}</pre>
</html>"""
    (directory / 'report.html').write_text(markup, encoding='utf-8')
    if result.get('validated'):
        (directory / 'regulator_parameters.yaml').write_text(
            yaml.safe_dump(result['selected_parameters'], sort_keys=False), encoding='utf-8')
