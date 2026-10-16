#!/bin/bash
# Ubuntu: 启动从臂跟随可视化 (RViz2, domain 1 隔离)
# 用法: ./scripts/start_follower_rviz.sh
set -e

cleanup() {
    echo "Stopping..."
    pkill -9 -f domain_bridge 2>/dev/null || true
    pkill -9 -f follower_visualizer 2>/dev/null || true
    pkill -9 -f 'ros2 launch rebotarm_gazebo follower_rviz' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

GAZEBO_WS="$HOME/Project/rebotarm-ros2-stack"

# 清理旧进程
echo "=== Cleaning up old processes ==="
pkill -9 -f domain_bridge 2>/dev/null || true
pkill -9 -f follower_visualizer 2>/dev/null || true
pkill -9 -f 'ros2 launch rebotarm_gazebo follower_rviz' 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source "$GAZEBO_WS/install/setup.bash"

echo "=== Starting domain bridge (domain 0 → domain 1) ==="
ros2 run joint_state_relay domain_bridge &
BRIDGE_PID=$!
sleep 2

echo "=== Starting follower arm visualization (domain 1) ==="
export ROS_DOMAIN_ID=1
ros2 launch rebotarm_gazebo follower_rviz.launch.py
