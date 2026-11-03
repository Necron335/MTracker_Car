"""Launch the car driver, joystick device and car joystick controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Expose device selection, motion limits and optional driver startup."""
    arguments = [
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('max_speed', default_value='0.5'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.26'),
        DeclareLaunchArgument('wheel_base', default_value='0.190'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('mtrackercar'), 'config', 'mtrackercar.yaml'])),
    ]
    geometry = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                for name in ('max_steering_angle', 'wheel_base')}
    return LaunchDescription(arguments + [
        Node(package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[LaunchConfiguration('config'), geometry,
                         {'command_source': 'cmd_vel'}], output='screen'),
        Node(package='joy', executable='joy_node', name='joy_node',
             condition=IfCondition(LaunchConfiguration('start_joy')),
             parameters=[{
                 'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                 'autorepeat_rate': 20.0,
             }], output='screen'),
        Node(package='mtrackercar', executable='mtrackercar_joy', name='mtrackercar_joy',
             parameters=[geometry, {
                 'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
                 'enable_motion': ParameterValue(
                     LaunchConfiguration('enable_motion'), value_type=bool),
             }], output='screen'),
    ])
