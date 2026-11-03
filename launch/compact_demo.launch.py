"""Repeated compact passes, steering changes in motion and a close RViz view."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def check_environment(context):
    """Explain missing overlay libraries before leaving RViz without a driver."""
    del context
    try:
        import ctypes
        from ackermann_msgs.msg import AckermannDriveStamped
        from rosidl_runtime_py.utilities import get_message
        ctypes.CDLL('libackermann_msgs__rosidl_typesupport_cpp.so')
        get_message('ackermann_msgs/msg/AckermannDriveStamped')
        AckermannDriveStamped()
    except (ImportError, OSError) as error:
        raise RuntimeError(
            'Ackermann environment unavailable. In the launch terminal run: '
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/student/ros2_workspace/install/setup.bash. '
            'Then restart the launch. Original error: '+str(error)) from error


def generate_launch_description():
    share = FindPackageShare('mtrackercar')
    driver = Node(
        package='mtrackercar', executable='mtrackercar_drv', name='mtrackercar',
        condition=IfCondition(LaunchConfiguration('start_driver')),
        parameters=[LaunchConfiguration('config'), {
            'command_source': 'steering_angle', 'steering_mode': 'angle',
            'max_steering_angle': 0.26, 'command_timeout': 0.15,
            'command_rate_hz': 100.0, 'odometry_topic': '/wheel/odom',
            'odom_frame_id': 'odom', 'frame_id': 'base_link',
            'odom_frame': 'odom', 'base_frame': 'base_link'}], output='screen')
    demo = Node(
        package='mtrackercar', executable='compact_demo', name='compact_demo',
        parameters=[{'enable_motion': ParameterValue(
            LaunchConfiguration('enable_motion'), value_type=bool), 'speed': 0.25}],
        remappings=[('odom', '/wheel/odom')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            share, 'config', 'mtrackercar.yaml'])),
        OpaqueFunction(function=check_environment),
        RegisterEventHandler(OnProcessExit(target_action=driver, on_exit=[EmitEvent(
            event=Shutdown(reason='Driver exited; stopping demo and RViz'))])),
        RegisterEventHandler(OnProcessExit(target_action=demo, on_exit=[EmitEvent(
            event=Shutdown(reason='Compact demo exited; stopping driver and RViz'))])),
        driver,
        demo,
        Node(package='mtrackercar', executable='trajectory_demo',
             name='compact_demo_trajectory',
             remappings=[('odom', '/wheel/odom'), ('trajectory', '/compact_demo/trajectory')],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'config', 'compact_demo.rviz'])],
             output='screen'),
    ])
