"""Launch raw-servo identification; motion additionally requires the start service."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Select exclusive raw driver input and configurable odom pose feedback."""
    share = FindPackageShare('mtrackercar')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            share, 'config', 'mtrackercar.yaml'])),
        DeclareLaunchArgument('calibration_config', default_value=PathJoinSubstitution([
            share, 'config', 'calibration.yaml'])),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('pose_topic', default_value='odom'),
        DeclareLaunchArgument('pose_type', default_value='odometry'),
        DeclareLaunchArgument('driver_odometry_topic', default_value='odom'),
        DeclareLaunchArgument('output_dir', default_value='calibration_results'),
        Node(package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[LaunchConfiguration('config'), {
                 'command_source': 'calibration_raw',
                 'odometry_topic': LaunchConfiguration('driver_odometry_topic'),
             }],
             output='screen'),
        Node(package='mtrackercar', executable='steering_calibration', name='steering_calibration',
             parameters=[LaunchConfiguration('calibration_config'), {
                 'enable_motion': ParameterValue(
                     LaunchConfiguration('enable_motion'), value_type=bool),
                 'pose_topic': LaunchConfiguration('pose_topic'),
                 'pose_type': LaunchConfiguration('pose_type'),
                 'output_dir': LaunchConfiguration('output_dir'),
             }], output='screen'),
    ])
