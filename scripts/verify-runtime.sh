#!/usr/bin/env bash
# Run identically in a host workspace and in the runtime image; mock MCU only.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
domain="${ROS_DOMAIN_ID:-221}"
[[ "$domain" =~ ^[0-9]+$ ]] && ((domain >= 220 && domain <= 232)) || { echo 'Verification requires isolated ROS_DOMAIN_ID 220..232' >&2; exit 2; }
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
cd "$root"
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 -m pytest src/joint_state_relay/test -v
python3 -m unittest discover -s scripts -p test_docker_runtime.py
python3 src/plugin/microros_stm32h723_bridge/test/smoke_trajectory_velocity.py
GRIPPER_SMOKE=1 python3 src/plugin/microros_stm32h723_bridge/test/smoke_trajectory_velocity.py
