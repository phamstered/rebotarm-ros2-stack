#!/bin/bash
# Jetson: 启动主臂 + 从臂驱动 + relay (real 模式)
# 所有节点在 ROS_DOMAIN_ID=0 下运行
set -e

cleanup() {
    echo "Stopping Jetson nodes..."
    pkill -f relay_node 2>/dev/null || true
    pkill -f digital_twin 2>/dev/null || true
    pkill -f robo_driver 2>/dev/null || true
    pkill -f stararm102_controller 2>/dev/null || true
    pkill -f follower_driver 2>/dev/null || true
    pkill -f robot_state_publisher 2>/dev/null || true
}
trap cleanup EXIT INT TERM

LEADER_WS="$HOME/Project/Star-Arm-102/ROS2_HUMBLE"
RELAY_WS="$HOME/Project/leader_follower"

source /opt/ros/humble/setup.bash

echo "=== Starting leader arm (digital twin + driver) ==="
source "$LEADER_WS/install/setup.bash"
ros2 launch stararm102_description digital_twin.launch.py \
    start_driver:=true use_rviz:=false &
LEADER_PID=$!
sleep 4

echo "=== Starting follower arm driver ==="
source "$RELAY_WS/install/setup.bash"
ros2 launch rebotarm_hw_driver follower_driver.launch.py \
    use_rviz:=false output_topic:=/follower/joint_states &
FOLLOWER_PID=$!
sleep 3

echo "=== Starting relay node (real mode) ==="
RELAY_CFG="$RELAY_WS/install/joint_state_relay/share/joint_state_relay/config/relay_real.yaml"
ros2 run joint_state_relay relay_node --ros-args \
    -p config_path:="$RELAY_CFG" &
RELAY_PID=$!
sleep 2

echo ""
echo "=== Jetson running (real mode) ==="
echo "  Leader: /joint_states (~31Hz, Star-Arm-102)"
echo "  Follower: /follower/joint_states (50Hz, reBotArm B601)"
echo "  Relay: /joint_states -> /joint_commands (100Hz)"
echo ""
echo "Press Ctrl+C to stop"

wait $LEADER_PID $FOLLOWER_PID $RELAY_PID
