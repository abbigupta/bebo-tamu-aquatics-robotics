"""Bring up ROS-owned guidance with an ArduPilot / MAVROS velocity backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial0:115200',
            description='MAVROS flight-controller connection URL.',
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }],
        ),
        Node(package='asv_control', executable='cmd_vel_mux', output='screen'),
        Node(
            package='asv_control',
            executable='mavros_velocity_adapter',
            output='screen',
        ),
        Node(
            package='asv_autonomy',
            executable='waypoint_follower',
            output='screen',
        ),
    ])
