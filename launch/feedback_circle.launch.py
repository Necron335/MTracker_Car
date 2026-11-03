"""Launch a TF feedback circle controller, driver and RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Expose reference dimensions, TF frame, gains and command limits."""
    numeric = {
        'radius': ('1.00', 'Circle radius in metres.'),
        'speed': ('0.10', 'Reference speed in m/s.'),
        'k_x': ('2.0', 'Longitudinal error gain (1/s).'),
        'k_y': ('40.0', 'Lateral error gain (1/m^2).'),
        'k_theta': ('10.0', 'Heading error gain (1/m).'),
        'max_linear': ('0.20', 'Absolute linear command limit in m/s.'),
        'max_angular': ('2.0', 'Absolute angular command limit in rad/s.'),
        'tf_timeout': ('0.25', 'Maximum TF age in seconds.'),
    }
    arguments = [DeclareLaunchArgument(name, default_value=value, description=description)
                 for name, (value, description) in numeric.items()]
    arguments.extend([
        DeclareLaunchArgument('enable_motion', default_value='false',
                              description='Enable physical robot motion.'),
        DeclareLaunchArgument('reference_frame', default_value='odom',
                              description='Fixed frame in which base_link is tracked.'),
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='Start the serial robot driver.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz with the demo view.'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('mtrackercar'), 'config', 'mtrackercar.yaml']),
            description='Driver parameter file.'),
    ])
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                  for name in numeric}
    parameters.update({
        'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool),
        'reference_frame': ParameterValue(LaunchConfiguration('reference_frame'), value_type=str),
        'base_frame': 'base_link',
    })
    return LaunchDescription(arguments + [
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
            package='mtrackercar', executable='controller_tracking',
            name='mtrackercar_controller_tracking', parameters=[parameters],
            remappings=[('trajectory', '/mtrackercar/trajectory'),
                        ('reference_path', '/mtrackercar/reference_path'),
                        ('reference_pose', '/mtrackercar/reference_pose')],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('mtrackercar'), 'config', 'circle_demo.rviz']),
                '-f', LaunchConfiguration('reference_frame')],
            output='screen',
        ),
    ])
