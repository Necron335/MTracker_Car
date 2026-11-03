"""Check circle launch configuration."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest


def load_module(relative_path):
    """Load a standalone installed script or launch source for testing."""
    path = Path(__file__).resolve().parents[1] / relative_path
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


circle = load_module('launch/circle_demo.launch.py')


@pytest.mark.parametrize('radius,speed', [
    ('0', '0.1'), ('-0.1', '0.1'), ('nan', '0.1'),
    ('0.1', 'inf'), ('0.1', '-0.1'), ('0.1', '0'),
])
def test_invalid_circle_parameters(radius, speed):
    """Reject invalid motion inputs before starting any nodes."""
    context = LaunchContext()
    context.launch_configurations.update(radius=radius, speed=speed)
    with pytest.raises(ValueError):
        circle.create_nodes(context)


@pytest.mark.parametrize('radius,expected_angular', [('0.1', 1.0), ('0.2', 0.5)])
def test_circle_motion_parameters(monkeypatch, radius, expected_angular):
    """Pass v/r to the controller and keep motion opt-in."""
    nodes = []
    original_node = circle.Node

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(circle, 'Node', capture_node)
    context = LaunchContext()
    context.launch_configurations.update(
        radius=radius, speed='0.1', enable_motion='false')
    assert len(circle.create_nodes(context)) == 4
    parameters = nodes[1]['parameters'][0]
    assert parameters['linear_velocity'] == 0.1
    assert parameters['angular_velocity'] == expected_angular
    assert parameters['enable_motion'].evaluate(context) is False


def test_feedback_launch_uses_cpp_and_tf_parameters(monkeypatch):
    """Select the C++ executable and pass typed TF feedback parameters."""
    from launch.actions import DeclareLaunchArgument

    feedback = load_module('launch/feedback_circle.launch.py')
    nodes = []
    original_node = feedback.Node

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(feedback, 'Node', capture_node)
    description = feedback.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['config'] = '/tmp/mtrackercar-test-config.yaml'
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert len(nodes) == 3
    assert nodes[1]['executable'] == 'controller_tracking'
    parameters = nodes[1]['parameters'][0]
    assert parameters['base_frame'] == 'base_link'
    assert parameters['reference_frame'].evaluate(context) == 'odom'
    assert parameters['enable_motion'].evaluate(context) is False
    assert parameters['radius'].evaluate(context) == 1.0
    assert parameters['speed'].evaluate(context) == 0.1
