"""Launch the car driver without assumed sensor or localization transforms."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Load car parameters and start the serial driver."""
    return LaunchDescription([
        DeclareLaunchArgument('command_source', default_value='steering_angle',
                              choices=['steering_angle', 'cmd_vel']),
        Node(
            package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
            parameters=[PathJoinSubstitution([
                FindPackageShare('mtrackercar'), 'config', 'mtrackercar.yaml']),
                {'command_source': LaunchConfiguration('command_source')}],
            output='screen'),
    ])
