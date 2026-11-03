"""Launch exclusive velocity experiments with explicit start/cancel services."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Use CMD=5 telemetry by default; support an optional firmware RAM provider."""
    share = FindPackageShare('mtrackercar')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            share, 'config', 'mtrackercar.yaml'])),
        DeclareLaunchArgument('experiment_config', default_value=PathJoinSubstitution([
            share, 'config', 'regulator_experiment.yaml'])),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('mode', default_value='record', choices=['record', 'tune']),
        DeclareLaunchArgument('odom_topic', default_value='/wheel/odom'),
        Node(package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[LaunchConfiguration('config'), {
                 'command_source': 'regulator_tuning', 'steering_mode': 'angle',
                 'odometry_topic': LaunchConfiguration('odom_topic')}],
             output='screen'),
        Node(package='mtrackercar', executable='regulator_tuning', name='regulator_tuning',
             arguments=['--config', LaunchConfiguration('experiment_config'),
                        '--mode', LaunchConfiguration('mode')],
             remappings=[('regulator/odom', LaunchConfiguration('odom_topic'))],
             output='screen'),
    ])
