"""Launch a 1 m radius circle demo, measured trajectory, and RViz."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def create_nodes(context):
    """Validate circle dimensions and compute angular velocity as v/r."""
    radius = float(LaunchConfiguration('radius').perform(context))
    speed = float(LaunchConfiguration('speed').perform(context))
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('radius must be finite and positive (metres)')
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('speed must be finite and positive (m/s)')
    return [
        Node(
            package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
            condition=IfCondition(LaunchConfiguration('start_driver')),
            parameters=[LaunchConfiguration('config'), {
                'command_source': 'cmd_vel',
                'odometry_topic': '/wheel/odom',
                'odom_frame_id': 'odom', 'frame_id': 'base_link',
            }],
            output='screen',
        ),
        Node(
            package='mtrackercar', executable='controller_simple',
            name='mtrackercar_controller_simple',
            parameters=[{
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'), value_type=bool),
                'linear_velocity': speed,
                'angular_velocity': speed / radius,
            }],
            output='screen',
        ),
        Node(
            package='mtrackercar', executable='trajectory_demo',
            name='mtrackercar_trajectory',
            remappings=[('odom', '/wheel/odom'),
                        ('trajectory', '/mtrackercar/trajectory')],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('mtrackercar'), 'config', 'circle_demo.rviz'])],
            output='screen',
        ),
    ]


def generate_launch_description():
    """Declare demo parameters and start its nodes."""
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.00',
                              description='Circle radius in metres.'),
        DeclareLaunchArgument('speed', default_value='0.10',
                              description='Forward speed in m/s.'),
        DeclareLaunchArgument('enable_motion', default_value='false',
                              description='Enable physical robot motion.'),
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='Start the serial robot driver.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz with the demo view.'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('mtrackercar'), 'config', 'mtrackercar.yaml']),
            description='Driver parameter file.'),
        OpaqueFunction(function=create_nodes),
    ])
