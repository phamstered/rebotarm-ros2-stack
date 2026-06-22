#!/bin/bash
# Jetson: 启动主臂 + relay 节点
set -e

cleanup() {
    echo "Stopping Jetson nodes..."
    pkill -f relay_node 2>/dev/null || true
    pkill -f digital_twin 2>/dev/null || true
    pkill -f robo_driver 2>/dev/null || true
    pkill -f stararm102_controller 2>/dev/null || true
}
trap cleanup EXIT INT TERM

LEADER_WS="$HOME/Project/Star-Arm-102/ROS2_HUMBLE"
RELAY_WS="$HOME/Project/leader_follower"

echo "=== Starting leader arm digital twin ==="
source /opt/ros/humble/setup.bash
source "$LEADER_WS/install/setup.bash"
ros2 launch stararm102_description digital_twin.launch.py \
    start_driver:=true use_rviz:=false &
LEADER_PID=$!
sleep 4

echo "=== Starting relay node ==="
source "$RELAY_WS/install/setup.bash"
ros2 run joint_state_relay relay_node &
RELAY_PID=$!
sleep 2

echo ""
echo "=== Jetson running ==="
echo "  /joint_states  (leader arm, ~31Hz)"
echo "  /joint_commands (relay output, 100Hz)"
echo ""
echo "Press Ctrl+C to stop"

wait $LEADER_PID $RELAY_PID
