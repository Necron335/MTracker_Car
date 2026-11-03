"""Verify bounded maneuvers using a bicycle plant, without ROS or hardware."""

import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'compact_demo', Path(__file__).resolve().parents[1]/'scripts/compact_demo.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_short_arcs_finish_and_remain_under_half_metre():
    engine = demo.Maneuvers(0.25)
    pose = (0.0, 0.0, 0.0)
    engine.start(pose, 0.0)
    velocity = 0.0
    driven = []
    last_moving = None
    stopped_between = False
    steering_changes = 0
    for tick in range(6000):
        now = tick*0.01
        engine.observe(pose, velocity, now)
        speed, angle = engine.command(now, velocity)
        if speed:
            if last_moving is not None:
                if math.copysign(1, last_moving[0]) != math.copysign(1, speed):
                    assert stopped_between, 'Direction changed without a stop'
                elif last_moving[1] != angle:
                    assert not stopped_between, 'Stopped just to change steering'
                    steering_changes += 1
            driven.append((math.copysign(1, speed), math.copysign(1, angle)))
            last_moving = speed, angle
            stopped_between = False
        else:
            stopped_between = True
        velocity += (1-math.exp(-0.01/0.08))*(speed-velocity)
        yaw_change = velocity*math.tan(angle)/0.19*0.01
        x, y, yaw = pose
        pose = (x+velocity*0.01*math.cos(yaw+yaw_change/2),
                y+velocity*0.01*math.sin(yaw+yaw_change/2), yaw+yaw_change)
        assert math.hypot(pose[0], pose[1]) < 0.5
        if engine.phase == 'completed':
            break
    assert engine.phase == 'completed'
    assert 4.5 < engine.distance < 5.5
    assert now >= 25
    assert set(driven) == {(1, 1), (-1, 1), (1, -1), (-1, -1)}
    assert steering_changes == 16
    assert engine.command(now+1, 0) == (0, 0)


def test_workspace_radius_remains_bounded():
    engine = demo.Maneuvers()
    engine.start((0, 0, 0), 0)
    with pytest.raises(ValueError, match='distance limit'):
        for i in range(1, 43):
            engine.observe((i*0.01, 0, 0), 0.1, i*0.1)


def test_final_stop_is_still_required():
    engine = demo.Maneuvers()
    engine.start((0, 0, 0), 0)
    for i in range(1, 32):
        engine.observe((i*0.01, 0, 0), 0.25, i*0.04)
        engine.command(i*0.04, 0.25)
    with pytest.raises(ValueError, match='did not stop'):
        engine.command(6, 0.25)


def test_wrong_direction_and_odometry_jump_abort():
    engine = demo.Maneuvers()
    engine.start((0, 0, 0), 0)
    engine.observe((0, 0, 0), 0, 0)
    engine.command(1, 0)
    with pytest.raises(ValueError, match='opposite'):
        engine.observe((-0.02, 0, 0), -0.25, 1.1)
    engine = demo.Maneuvers()
    engine.start((0, 0, 0), 0)
    with pytest.raises(ValueError, match='discontinuity'):
        engine.observe((0.2, 0, 0), 0, 0.1)
