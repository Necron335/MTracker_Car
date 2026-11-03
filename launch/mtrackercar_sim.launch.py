"""Run the kinematic car simulator with optional trajectory view."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse driver configuration and publish the same odometry and TF."""
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('mtrackercar'), 'config', 'mtrackercar.yaml'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='mtrackercar', executable='mtrackercar_sim', name='mtrackercar',
             parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='mtrackercar', executable='trajectory_demo', name='mtrackercar_trajectory',
             condition=IfCondition(LaunchConfiguration('rviz')),
             remappings=[('odom', '/wheel/odom'), ('trajectory', '/mtrackercar/trajectory')],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('mtrackercar'), 'config', 'circle_demo.rviz'])],
             output='screen'),
    ])
