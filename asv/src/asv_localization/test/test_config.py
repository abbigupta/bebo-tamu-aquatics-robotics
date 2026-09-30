from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).parents[1] / 'config'


def parameters(filename: str, node_name: str) -> dict:
    path = CONFIG_DIRECTORY / filename
    return yaml.safe_load(path.read_text(encoding='utf-8'))[node_name][
        'ros__parameters'
    ]


def test_local_filter_uses_motion_reference_and_imu():
    config = parameters('ekf_local.yaml', 'ekf_local_node')
    assert config['odom0'] == '/asv/odometry/ground_truth'
    assert config['imu0'] == '/asv/imu/data'
    assert config['world_frame'] == 'odom'


def test_global_filter_uses_gps_and_local_odometry():
    config = parameters('ekf_global.yaml', 'ekf_global_node')
    assert config['odom0'] == '/asv/odometry/gps'
    assert config['odom1'] == '/asv/odometry/local'
    assert config['world_frame'] == 'map'
