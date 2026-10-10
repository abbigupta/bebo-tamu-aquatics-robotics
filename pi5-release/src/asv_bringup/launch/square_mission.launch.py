"""Run a square mission against live Pixhawk odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side_length_m', default_value='5.0'),
        DeclareLaunchArgument('cruise_speed_mps', default_value='0.7'),
        DeclareLaunchArgument('acceptance_radius_m', default_value='0.5'),
        DeclareLaunchArgument(
            'odometry_topic', default_value='/mavros/local_position/odom'
        ),
        Node(
            package='asv_autonomy',
            executable='square_mission',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'side_length_m': LaunchConfiguration('side_length_m'),
                'cruise_speed_mps': LaunchConfiguration('cruise_speed_mps'),
                'acceptance_radius_m': LaunchConfiguration(
                    'acceptance_radius_m'
                ),
                'odometry_topic': LaunchConfiguration('odometry_topic'),
            }],
        ),
    ])
