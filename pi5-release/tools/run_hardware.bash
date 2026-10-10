#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
source "${workspace_dir}/install/setup.bash"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  printf '%s\n' \
    'MAVROS is not installed for ROS 2 Lyrical.' \
    'Install the ros-lyrical-mavros package and retry.' >&2
  exit 1
fi

exec ros2 launch asv_bringup hardware.launch.py "$@"
