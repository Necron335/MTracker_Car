"""Launch the optional constant-velocity demo controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare motion settings and launch the demo controller."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_motion', default_value='false',
            description='Enable publishing nonzero motion commands.',
        ),
        DeclareLaunchArgument(
            'linear_velocity', default_value='0.05',
            description='Forward velocity in m/s.',
        ),
        DeclareLaunchArgument(
            'angular_velocity', default_value='0.2',
            description='Angular velocity in rad/s.',
        ),
        Node(
            package='mtrackercar',
            executable='controller_simple',
            name='mtrackercar_controller_simple',
            parameters=[{
                'enable_motion': ParameterValue(
                    LaunchConfiguration('enable_motion'), value_type=bool),
                'linear_velocity': ParameterValue(
                    LaunchConfiguration('linear_velocity'), value_type=float),
                'angular_velocity': ParameterValue(
                    LaunchConfiguration('angular_velocity'), value_type=float),
            }],
            output='screen',
        ),
    ])
