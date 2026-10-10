"""Bring up the Raspberry Pi 5 and Pixhawk hardware stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial0:115200',
            description='MAVROS flight-controller connection URL.',
        ),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument(
            'odometry_topic',
            default_value='/mavros/local_position/odom',
            description='Pixhawk/MAVROS odometry consumed by autonomy.',
        ),
        DeclareLaunchArgument('start_tfmini', default_value='false'),
        DeclareLaunchArgument('tfmini_port', default_value='/dev/ttyAMA3'),
        DeclareLaunchArgument('start_bme280', default_value='false'),
        DeclareLaunchArgument('bme280_address', default_value='118'),
        DeclareLaunchArgument('start_ds18b20', default_value='false'),
        DeclareLaunchArgument(
            'ds18b20_device_id', default_value='28-000000230f20'
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
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
            parameters=[{
                'odometry_topic': LaunchConfiguration('odometry_topic'),
            }],
        ),
        Node(
            package='asv_sensors',
            executable='tfmini_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_tfmini')),
            parameters=[{'port': LaunchConfiguration('tfmini_port')}],
        ),
        Node(
            package='asv_sensors',
            executable='bme280_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_bme280')),
            parameters=[{
                'address': ParameterValue(
                    LaunchConfiguration('bme280_address'), value_type=int
                ),
            }],
        ),
        Node(
            package='asv_sensors',
            executable='ds18b20_driver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_ds18b20')),
            parameters=[{
                'device_id': LaunchConfiguration('ds18b20_device_id'),
            }],
        ),
    ])
