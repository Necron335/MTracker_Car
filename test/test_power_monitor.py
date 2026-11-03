"""Test voltage decisions without ROS, sudo, or actual shutdown."""

import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'power_monitor', Path(__file__).resolve().parents[1] / 'scripts/power_monitor.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def guard():
    return module.VoltageGuard(10.0, 0.3, 2.0, 1.0, 0.75)


def test_continuous_low_voltage_and_countdown():
    monitor = guard()
    assert monitor.state(0) == 'unknown'
    for tick in range(7):
        now = tick*0.5
        monitor.sample(9.9, now)
        state = monitor.state(now)
        assert state == ('low' if now < 2 else 'countdown' if now < 3 else 'shutdown')


def test_transient_dip_does_not_shutdown():
    monitor = guard()
    monitor.sample(9.8, 0)
    assert monitor.state(0.5) == 'low'
    monitor.sample(10.4, 0.5)
    assert monitor.state(0.5) == 'normal'


def test_recovery_cancels_countdown():
    monitor = guard()
    for tick in range(5):
        monitor.sample(9.9, tick*0.5)
    assert monitor.state(2) == 'countdown'
    monitor.sample(10.4, 2.5)
    assert monitor.state(2.5) == 'normal'


def test_no_samples_never_trigger_shutdown():
    monitor = guard()
    monitor.sample(9, 0)
    assert monitor.state(4) == 'unknown'
    monitor.sample(9, 5)
    assert monitor.state(5) == 'low'


@pytest.mark.parametrize('voltage', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_voltage_is_unknown(voltage):
    monitor = guard()
    monitor.sample(voltage, 0)
    assert monitor.state(0) == 'unknown'
