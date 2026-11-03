"""Verify reference profiles, firmware-buffer integrity and safe tuning commits."""

from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
try:
    import regulator_experiment as experiment
    import regulator_tuning as tuning
finally:
    sys.path.pop(0)

BOUNDS = [[10, 20], [60, 100], [0.8, 1.2], [0.002, 0.003]]


@pytest.mark.parametrize('persistent', [False, True])
def test_settling_rejection_continues_validation(tmp_path, persistent):
    """A bad settling window cannot discard all completed training trials."""
    class SettlingBackend(tuning.TelemetrySyntheticBackend):
        def __init__(self):
            super().__init__()
            self.evaluations = 0
            self.recoveries = 0

        def evaluate(self, *args):
            self.evaluations += 1
            if self.evaluations == 13 or (persistent and self.evaluations > 12):
                raise experiment.TrialQualityRejected('Robot did not settle after the final stop')
            return experiment.evaluate_telemetry(*args)

        def wait_stopped(self):
            self.recoveries += 1

    backend = SettlingBackend()
    engine = experiment.TuneRunner(backend, tmp_path / 'settle',
                                   experiment.FIRMWARE_DEFAULTS, BOUNDS, budget=12)
    result = engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19),
                        apply=True, persist=True)
    assert len(engine.trials) == (20 if persistent else 17)
    assert backend.recoveries == (8 if persistent else 1)
    assert all(t['accepted'] for t in engine.trials[:12])
    if persistent:
        assert not result['validated'] and not backend.saved
        assert result['relative_improvement'] is None
        np.testing.assert_allclose(backend.parameters, experiment.FIRMWARE_DEFAULTS)
    else:
        assert result['validated'] and result['applied'] and backend.saved


def test_settling_recovery_cannot_ignore_continued_motion(tmp_path):
    """A robot that keeps moving must not receive another experiment."""
    class MovingBackend(tuning.TelemetrySyntheticBackend):
        def evaluate(self, *args):
            raise experiment.TrialQualityRejected('Robot did not settle after the final stop')

        def wait_stopped(self):
            raise RuntimeError('Robot still moving')

    backend = MovingBackend()
    engine = experiment.TuneRunner(backend, tmp_path / 'moving',
                                   experiment.FIRMWARE_DEFAULTS, BOUNDS, budget=12)
    with pytest.raises(RuntimeError, match='still moving'):
        engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19))
    assert len(engine.trials) == 1
    assert not backend.saved


@pytest.mark.parametrize('fault', ['gap', 'short', 'sign', 'saturation', 'reference', 'nan'])
def test_telemetry_faults_abort_evaluation(fault):
    """Do not optimize corrupted or unsafe host telemetry."""
    profile = experiment.Profile()
    backend = tuning.TelemetrySyntheticBackend()
    capture = backend.capture(profile)
    rows = np.array(capture['rows'])
    if fault == 'gap':
        rows[10:, 1] += 0.3
    elif fault == 'short':
        rows = rows[:-100]
    elif fault == 'sign':
        rows[:, 3] *= -1
    elif fault == 'saturation':
        rows[:, 4] = 1
    elif fault == 'reference':
        rows[:, 2] = 0
    else:
        rows[10, 3] = np.nan
    with pytest.raises(ValueError):
        experiment.evaluate_telemetry(rows, profile, 0.01)


def test_telemetry_search_report_and_no_implicit_apply(tmp_path):
    """Complete the search using telemetry-only scoring and an honest CSV schema."""
    backend = tuning.TelemetrySyntheticBackend()
    engine = experiment.TuneRunner(backend, tmp_path / 'telemetry',
                                   experiment.FIRMWARE_DEFAULTS, BOUNDS, budget=12)
    result = engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19))
    assert result['validated'] and not result['applied'] and not result['persisted']
    np.testing.assert_allclose(backend.parameters, experiment.FIRMWARE_DEFAULTS)
    document = yaml.safe_load((engine.directory / 'tuning.yaml').read_text())
    assert document['acquisition'] == 'cmd5_telemetry'
    csv = (engine.directory / 'trial_000.csv').read_text()
    assert csv.splitlines()[0] == ','.join(experiment.TELEMETRY_COLUMNS)
    assert 'left_position_error_rad' not in csv
    experiment.write_report(engine.directory)
    assert 'cmd5_telemetry' in (engine.directory / 'report.html').read_text()


def test_cancel_before_commit_restores_baseline(tmp_path):
    """Cancellation arriving after the last capture cannot commit best values."""
    class CancelAtCommit(tuning.TelemetrySyntheticBackend):
        def __init__(self):
            super().__init__()
            self.checks = 0

        def check_feedback(self):
            self.checks += 1
            if self.checks == 17:  # 12 training + 4 validation, then commit
                raise RuntimeError('Cancelled by operator')

    backend = CancelAtCommit()
    engine = experiment.TuneRunner(backend, tmp_path / 'cancel',
                                   experiment.FIRMWARE_DEFAULTS, BOUNDS, budget=12)
    with pytest.raises(RuntimeError, match='Cancelled'):
        engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19), apply=True)
    np.testing.assert_allclose(backend.parameters, experiment.FIRMWARE_DEFAULTS)
    assert not backend.saved


@pytest.mark.parametrize('key,value', [('enable_motion', 'false'), ('persist_best', 'false'),
                                     ('max_trials', 1.5), ('validation_repeats', True)])
def test_invalid_authorization_and_budget_types(key, value):
    """Strings such as 'false' must never authorize hardware actions."""
    settings = yaml.safe_load((SCRIPTS.parent / 'config/regulator_experiment.yaml').read_text())
    settings[key] = value
    with pytest.raises(ValueError):
        tuning.validate_settings(settings)


@pytest.mark.parametrize('kind', ['steps', 'ramp', 'sine', 'chirp', 'prbs'])
def test_profiles_are_bounded_reproducible_and_stop(kind):
    """Exercise both directions without leaving a nonzero final command."""
    profile = experiment.Profile(kind=kind).validate()
    samples = [profile.value(t) for t in np.linspace(0, profile.duration, 1001)]
    assert max(samples) > 0 and min(samples) < 0
    assert max(abs(v) for v in samples) <= profile.amplitude
    assert profile.value(0) == profile.value(profile.duration) == 0
    assert profile.value(-1) == profile.value(profile.duration+1) == 0
    assert samples == [profile.value(t) for t in np.linspace(0, profile.duration, 1001)]


@pytest.mark.parametrize('fault', ['overflow', 'index', 'nan', 'short', 'sign', 'output'])
def test_bad_buffer_cannot_be_used_to_tune(fault):
    """Reject invalid data instead of replacing it with host-derived telemetry."""
    profile = experiment.Profile()
    capture = tuning.SyntheticBackend().capture(profile)
    rows = np.array(capture['rows'])
    if fault == 'overflow':
        with pytest.raises(ValueError):
            experiment.validate_samples(rows, profile, 0.01, overflow=True)
        return
    if fault == 'index':
        rows[10, 0] += 1
    elif fault == 'nan':
        rows[10, 4] = np.nan
    elif fault == 'short':
        rows = rows[:100]
    elif fault == 'sign':
        rows[:, 2] *= -1
    else:
        rows[10, 5] = 12
    with pytest.raises(ValueError):
        experiment.evaluate(rows, profile, 0.01)


class AuditedBackend(tuning.SyntheticBackend):
    """Expose trial ordering and inject failures without touching hardware."""

    def __init__(self, fail_capture=False, fail_save=False):
        super().__init__()
        self.events = []
        self.fail_capture = fail_capture
        self.fail_save = fail_save

    def stop(self):
        """Record stop-before-write and stop-before-save ordering."""
        self.events.append('stop')

    def set_parameters(self, values):
        """Record each complete four-value transaction."""
        self.events.append(('set', tuple(values)))
        super().set_parameters(values)

    def capture(self, profile):
        """Return a synthetic capture or emulate data loss."""
        self.events.append('capture')
        if self.fail_capture:
            raise RuntimeError('buffer disconnected')
        return super().capture(profile)

    def save(self):
        """Record an EEPROM transaction and optionally emulate an unknown result."""
        self.events.append('save')
        if self.fail_save:
            raise RuntimeError('EEPROM timeout; result unknown')
        super().save()


def runner(tmp_path, backend, **kwargs):
    """Create a bounded search with an explicit, known baseline."""
    return experiment.TuneRunner(backend, tmp_path / 'run', experiment.FIRMWARE_DEFAULTS,
                                 BOUNDS, budget=12, **kwargs)


def test_tunes_validates_and_saves_only_selected_values(tmp_path):
    """A hardware commit requires independent repeated validation improvement."""
    backend = AuditedBackend()
    engine = runner(tmp_path, backend)
    result = engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19),
                        apply=True, persist=True)
    assert result['validated'] and result['applied'] and result['persisted']
    assert result['relative_improvement'] >= 0.03
    assert backend.saved
    assert backend.events.count('save') == 1
    assert len([r for r in engine.trials if r['phase'].startswith('validation')]) == 4
    save_index = backend.events.index('save')
    assert backend.events[save_index-1][0] == 'set'
    for index, event in enumerate(backend.events):
        if isinstance(event, tuple) and event[0] == 'set':
            assert backend.events[index-1] == 'stop'
    experiment.write_report(engine.directory)
    assert (engine.directory / 'regulator_parameters.yaml').exists()
    assert 'data:image/png;base64,' in (engine.directory / 'report.html').read_text()


def test_rejected_candidate_restores_baseline_without_eeprom(tmp_path):
    """No measurable validation improvement means no candidate commit."""
    backend = AuditedBackend()
    engine = experiment.TuneRunner(backend, tmp_path / 'run', experiment.FIRMWARE_DEFAULTS,
                                   BOUNDS, budget=1)
    result = engine.run(experiment.Profile(), experiment.Profile(kind='prbs'),
                        apply=True, persist=True)
    assert not result['validated'] and not result['applied'] and not backend.saved
    np.testing.assert_allclose(backend.parameters, experiment.FIRMWARE_DEFAULTS)


def test_loss_of_buffer_aborts_and_rolls_back(tmp_path):
    """A failed acquisition never triggers another motion or EEPROM write."""
    backend = AuditedBackend(fail_capture=True)
    engine = runner(tmp_path, backend)
    with pytest.raises(RuntimeError, match='buffer disconnected'):
        engine.run(experiment.Profile(), experiment.Profile(kind='prbs'), apply=True, persist=True)
    assert backend.events.count('capture') == 1
    assert not backend.saved
    np.testing.assert_allclose(backend.parameters, experiment.FIRMWARE_DEFAULTS)
    result = yaml.safe_load((engine.directory / 'tuning.yaml').read_text())['result']
    assert result['baseline_restored']


def test_save_timeout_records_unknown_eeprom_outcome(tmp_path):
    """A lost save response cannot be reported as either successful or not written."""
    backend = AuditedBackend(fail_save=True)
    engine = runner(tmp_path, backend)
    with pytest.raises(RuntimeError, match='EEPROM timeout'):
        engine.run(experiment.Profile(), experiment.Profile(kind='prbs', seed=19),
                   apply=True, persist=True)
    result = yaml.safe_load((engine.directory / 'tuning.yaml').read_text())['result']
    assert result['persistence_outcome'] == 'unknown'
    assert not result['persisted']


def test_record_does_not_change_regulator_or_save(tmp_path):
    """Recording can inspect the current controller without changing its settings."""
    settings = yaml.safe_load((SCRIPTS.parent / 'config/regulator_experiment.yaml').read_text())
    backend = AuditedBackend()
    tuning.run_experiment(backend, settings, tmp_path / 'record', 'record')
    assert not any(isinstance(e, tuple) and e[0] == 'set' for e in backend.events)
    assert not backend.saved
    assert (tmp_path / 'record/trial_000.csv').exists()
    assert (tmp_path / 'record/report.html').exists()


def test_search_bounds_checked_before_writes(tmp_path):
    """Never send candidates outside the configured positive parameter region."""
    backend = AuditedBackend()
    with pytest.raises(ValueError):
        experiment.TuneRunner(backend, tmp_path / 'bad', [100, 80, 1, 0.0025], BOUNDS)
    assert not backend.events


def test_documented_ram_layout_and_car_channel_units():
    """Preserve requested channel order and signed values of actual CAR signals."""
    import struct
    sys.path.insert(0, str(SCRIPTS))
    try:
        from trace_data import car_capture_rows, decode_ram_trace
    finally:
        sys.path.pop(0)
    ids = [15, 13, 5, 1, 11, 10]
    raw = (struct.pack('<ffffiI', 2.0, -0.125, -0.4, 1.5, -200, 12000) +
           struct.pack('<ffffiI', -2.0, 0.25, 0.5, -1.5, 50, 11900))
    words = list(struct.unpack('<' + 'H'*(len(raw)//2), raw))
    decoded = decode_ram_trace(words, ids)
    assert decoded[0]['leftPositionError'] == -0.125
    assert decoded[0]['ina219CurrentDeciMa'] == -200
    rows = car_capture_rows(words, ids, 1, 0.033)
    np.testing.assert_allclose(rows[0], [0, 0, 0.066, 0.0495, -0.125, -0.4], atol=1e-7)
    np.testing.assert_allclose(rows[1], [1, 0.001, -0.066, -0.0495, 0.25, 0.5])
    with pytest.raises(ValueError):
        car_capture_rows(words, [2, 14, 6, 16, 11, 10], 1, 0.033)
    with pytest.raises(ValueError):
        decode_ram_trace(words[:-1], ids)
