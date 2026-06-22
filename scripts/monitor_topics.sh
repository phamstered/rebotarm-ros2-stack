#!/bin/bash
# 本机: 监控双端 topic 数据流
set -e

echo "=== Jetson topics ==="
echo "  /joint_states   (主臂原始)"
echo "  /joint_commands  (中继后)"
echo ""
echo "=== Ubuntu topics ==="
echo "  /joint_commands  (跨机器 DDS)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ssh damiao "source /opt/ros/humble/setup.bash && ros2 topic hz /joint_states /joint_commands" &
JETSON_HZ=$!

ssh ubuntu "source /opt/ros/humble/setup.bash && ros2 topic hz /joint_commands" &
UBUNTU_HZ=$!

trap "kill $JETSON_HZ $UBUNTU_HZ 2>/dev/null" EXIT INT TERM
wait
