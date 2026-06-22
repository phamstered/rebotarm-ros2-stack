#!/bin/bash
# Jetson: 启动主臂 + relay
# 用法: ./scripts/start_jetson.sh [--mode=gazebo|real]
#   gazebo: 仅主臂 + relay (从臂在 Ubuntu Gazebo 仿真)
#   real:   主臂 + 从臂驱动 + relay (从臂为真实硬件)

MODE="gazebo"
for arg in "$@"; do
    case $arg in
        --mode=*) MODE="${arg#*=}" ;;
    esac
done

_CLEANUP_DONE=0
cleanup() {
    if [ "$_CLEANUP_DONE" = "1" ]; then return; fi
    _CLEANUP_DONE=1
    echo ""
    echo "=== Stopping Jetson nodes ==="
    if [ "$MODE" = "real" ]; then
        echo "  Stopping follower driver (return to zero + disable)..."
        # 发 SIGINT 触发归零流程 (follower_driver 忽略 SIGTERM)
        pkill -2 -f follower_driver 2>/dev/null || true
        # 等待退出，最多 8 秒
        for i in $(seq 1 8); do
            if ! pgrep -f follower_driver > /dev/null 2>&1; then
                echo "  Follower driver exited."
                break
            fi
            sleep 1
        done
        if pgrep -f follower_driver > /dev/null 2>&1; then
            echo "  Follower driver still running, force killing..."
            pkill -9 -f follower_driver 2>/dev/null || true
        fi
    fi
    pkill -f relay_node 2>/dev/null || true
    pkill -f digital_twin 2>/dev/null || true
    pkill -f robo_driver 2>/dev/null || true
    pkill -f stararm102_controller 2>/dev/null || true
    pkill -f robot_state_publisher 2>/dev/null || true
    echo "  All stopped."
}
trap cleanup EXIT
trap 'exit 0' INT TERM

LEADER_WS="$HOME/Project/Star-Arm-102/ROS2_HUMBLE"
RELAY_WS="$HOME/Project/leader_follower"

source /opt/ros/humble/setup.bash

# ── CAN 总线初始化 (从臂电机需要) ──
if [ "$MODE" = "real" ]; then
    echo "=== Initializing CAN bus (can0, 500kbps) ==="
    if ! ip link show can0 &>/dev/null; then
        sudo ip link set can0 type can bitrate 500000 2>/dev/null || true
    fi
    if ip link show can0 | grep -q "state DOWN"; then
        sudo ip link set can0 up
        echo "  can0 is UP"
    else
        echo "  can0 already up"
    fi
fi

echo "=== Starting leader arm (digital twin + driver) ==="
source "$LEADER_WS/install/setup.bash"
ros2 launch stararm102_description digital_twin.launch.py \
    start_driver:=true use_rviz:=false &
LEADER_PID=$!
sleep 4

source "$RELAY_WS/install/setup.bash"

if [ "$MODE" = "real" ]; then
    echo "=== Starting follower arm driver (real hardware) ==="
    # 用 setsid 创建新进程组，避免 Ctrl+C 直接发到 follower
    setsid ros2 launch rebotarm_hw_driver follower_driver.launch.py \
        use_rviz:=false output_topic:=/follower/joint_states &
    FOLLOWER_PID=$!
    sleep 3

    echo "=== Checking follower motor status ==="
    if timeout 3 ros2 topic echo /follower/joint_states --once > /dev/null 2>&1; then
        echo "  Follower motors ENABLED"
    else
        echo "  WARNING: Follower joint_states not publishing!"
    fi

    echo "=== Starting relay node (real mode) ==="
    RELAY_CFG="$RELAY_WS/install/joint_state_relay/share/joint_state_relay/config/relay_real.yaml"
    ros2 run joint_state_relay relay_node --ros-args \
        -p config_path:="$RELAY_CFG" &
    RELAY_PID=$!
else
    echo "=== Starting relay node (gazebo mode) ==="
    RELAY_CFG="$RELAY_WS/install/joint_state_relay/share/joint_state_relay/config/relay.yaml"
    ros2 run joint_state_relay relay_node --ros-args \
        -p config_path:="$RELAY_CFG" &
    RELAY_PID=$!
fi

sleep 2

echo ""
echo "=== Jetson running ($MODE mode) ==="
echo "  Leader: /joint_states (~31Hz, Star-Arm-102)"
if [ "$MODE" = "real" ]; then
    echo "  Follower: /follower/joint_states (50Hz, reBotArm B601)"
fi
echo "  Relay: /joint_states -> /joint_commands (100Hz)"
echo ""
echo "Press Ctrl+C to stop"

wait
