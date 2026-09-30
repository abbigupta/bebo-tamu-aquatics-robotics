"""Launch the ASV in a coastal-water Gazebo Sim world."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml  # type: ignore[import-untyped]


def _simulation_nodes(context):
    description_share = Path(get_package_share_directory('asv_description'))
    localization_share = Path(
        get_package_share_directory('asv_localization')
    )
    model_file = description_share / 'urdf' / 'asv.urdf.xacro'
    physics_file = Path(LaunchConfiguration('physics_config').perform(context))
    with physics_file.open(encoding='utf-8') as stream:
        physics = yaml.safe_load(stream)['model']
    current_speed = float(
        LaunchConfiguration('current_speed_mps').perform(context)
    )
    current_direction_deg = float(
        LaunchConfiguration('current_direction_deg').perform(context)
    )
    current_direction = math.radians(current_direction_deg)
    mappings = {
        'use_mesh': LaunchConfiguration('use_mesh').perform(context),
        'mesh_file': LaunchConfiguration('mesh_file').perform(context),
        'mesh_scale': LaunchConfiguration('mesh_scale').perform(context),
        'mesh_x': LaunchConfiguration('mesh_x').perform(context),
        'mesh_y': LaunchConfiguration('mesh_y').perform(context),
        'mesh_z': LaunchConfiguration('mesh_z').perform(context),
        'mesh_roll': LaunchConfiguration('mesh_roll').perform(context),
        'mesh_pitch': LaunchConfiguration('mesh_pitch').perform(context),
        'mesh_yaw': LaunchConfiguration('mesh_yaw').perform(context),
        'current_x': str(current_speed * math.cos(current_direction)),
        'current_y': str(current_speed * math.sin(current_direction)),
        'current_z': LaunchConfiguration(
            'current_vertical_speed_mps'
        ).perform(context),
    }
    mappings.update({name: str(value) for name, value in physics.items()})
    description = xacro.process_file(
        str(model_file),
        mappings=mappings,
    ).toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': description},
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_asv',
            output='screen',
            parameters=[{
                'world': 'calm_water',
                'string': description,
                'name': 'asv',
                'allow_renaming': False,
                'x': 0.0,
                'y': 0.0,
                'z': 0.10,
                'R': 0.0,
                'P': 0.0,
                'Y': 0.0,
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='simulation_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/asv/joint/port_thruster_joint/cmd_thrust'
                '@std_msgs/msg/Float64]gz.msgs.Double',
                '/model/asv/joint/starboard_thruster_joint/cmd_thrust'
                '@std_msgs/msg/Float64]gz.msgs.Double',
                '/model/asv/ocean_current'
                '@geometry_msgs/msg/Vector3]gz.msgs.Vector3d',
                '/model/asv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/asv/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/asv/gps/fix_raw@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/asv/lidar/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/asv/sonar/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            remappings=[
                ('/model/asv/odometry', '/asv/odometry/ground_truth'),
            ],
        ),
        Node(
            package='asv_sensors',
            executable='gps_covariance_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='asv_sensors',
            executable='sonar_range_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='asv_control',
            executable='twin_thruster_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'max_forward_thrust_n': 25.0},
                {'max_reverse_thrust_n': 15.0},
                {'command_timeout_s': 0.75},
            ],
        ),
        Node(
            package='asv_control',
            executable='ocean_current',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mean_speed_mps': current_speed,
                'mean_direction_deg': current_direction_deg,
                'vertical_speed_mps': float(LaunchConfiguration(
                    'current_vertical_speed_mps'
                ).perform(context)),
                'speed_noise_stddev': float(LaunchConfiguration(
                    'current_speed_noise_stddev'
                ).perform(context)),
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[
                str(
                    localization_share
                    / 'config'
                    / 'ekf_local.yaml'
                ),
                {'use_sim_time': True},
            ],
            remappings=[
                ('/odometry/filtered', '/asv/odometry/local'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                str(
                    localization_share
                    / 'config'
                    / 'navsat_transform.yaml'
                ),
                {'use_sim_time': True},
            ],
            remappings=[
                ('/odometry/filtered', '/asv/odometry/local'),
                ('/gps/fix', '/asv/gps/fix'),
                ('/imu', '/asv/imu/data'),
                ('/odometry/gps', '/asv/odometry/gps'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[
                str(
                    localization_share
                    / 'config'
                    / 'ekf_global.yaml'
                ),
                {'use_sim_time': True},
            ],
            remappings=[
                ('/odometry/filtered', '/asv/odometry/filtered'),
            ],
        ),
        Node(
            package='asv_control',
            executable='cmd_vel_mux',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='asv_control',
            executable='velocity_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'thruster_offset_y_m': float(physics['thruster_offset_y'])},
            ],
        ),
        Node(
            package='asv_autonomy',
            executable='waypoint_follower',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]


def generate_launch_description():
    description_share = Path(get_package_share_directory('asv_description'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))
    world_file = description_share / 'worlds' / 'calm_water.sdf'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                LaunchConfiguration('gz_args_extra'),
                f' -r -v 3 {world_file}',
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'physics_config',
            default_value=str(
                description_share / 'config' / 'physics_sim.yaml'
            ),
            description='YAML file containing vessel physics parameters.',
        ),
        DeclareLaunchArgument(
            'use_mesh',
            default_value='false',
            description='Use a detailed visual mesh instead of preview shapes.',
        ),
        DeclareLaunchArgument(
            'mesh_file',
            default_value='asv_visual.obj',
            description='Filename under asv_description/meshes/visual.',
        ),
        DeclareLaunchArgument(
            'mesh_scale',
            default_value='0.001',
            description='Uniform STL scale; 0.001 converts millimetres to metres.',
        ),
        DeclareLaunchArgument('mesh_x', default_value='0.0'),
        DeclareLaunchArgument('mesh_y', default_value='0.0'),
        DeclareLaunchArgument('mesh_z', default_value='0.0'),
        DeclareLaunchArgument('mesh_roll', default_value='0.0'),
        DeclareLaunchArgument('mesh_pitch', default_value='0.0'),
        DeclareLaunchArgument('mesh_yaw', default_value='1.57079632679'),
        DeclareLaunchArgument(
            'gz_args_extra',
            default_value='',
            description='Additional Gazebo arguments, for example -s for server only.',
        ),
        DeclareLaunchArgument(
            'current_speed_mps',
            default_value='0.25',
            description='Mean horizontal ocean-current speed in m/s.',
        ),
        DeclareLaunchArgument(
            'current_direction_deg',
            default_value='30.0',
            description='Current direction in the world ENU frame (degrees).',
        ),
        DeclareLaunchArgument(
            'current_vertical_speed_mps',
            default_value='0.0',
            description='Mean vertical current; positive is upward.',
        ),
        DeclareLaunchArgument(
            'current_speed_noise_stddev',
            default_value='0.015',
            description='Gauss-Markov current speed noise in m/s/sqrt(s).',
        ),
        gazebo,
        OpaqueFunction(function=_simulation_nodes),
    ])
