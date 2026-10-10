#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
source "${workspace_dir}/install/setup.bash"
exec ros2 run asv_control keyboard_teleop "$@"
