#!/bin/bash
# Ubuntu: 启动 Gazebo Fortress 仿真 (ign-gazebo6 + ros2_control)
# 用法: ./scripts/start_gazebo.sh [--headless=True|False]

HEADLESS="False"
for arg in "$@"; do
    case $arg in
        --headless=*) HEADLESS="${arg#*=}" ;;
    esac
done

GAZEBO_WS="$HOME/Project/leader_follower"
MODEL_DIR="$GAZEBO_WS/install/rebotarm_gazebo/share/rebotarm_gazebo/models"

# ── 停止所有旧仿真/MoveIt/可视化进程 ──
echo "=== Stopping old processes ==="
pkill -9 -f 'ign gazebo' 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
pkill -9 -f spawner 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f domain_bridge 2>/dev/null || true
pkill -9 -f relay_node 2>/dev/null || true
pkill -9 -f follower_visualizer 2>/dev/null || true
sleep 1

cleanup() {
    echo "Stopping Gazebo..."
    pkill -9 -f 'ign gazebo' 2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f controller_manager 2>/dev/null || true
    pkill -9 -f spawner 2>/dev/null || true
    pkill -9 -f domain_bridge 2>/dev/null || true
    pkill -9 -f relay_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$HEADLESS" = "True" ] || [ "$HEADLESS" = "true" ]; then
    if ! pgrep -x Xvfb > /dev/null; then
        echo "Starting Xvfb (headless mode)..."
        Xvfb :99 -screen 0 1024x768x24 &
        sleep 1
    fi
    export DISPLAY=:99
fi

if [ -z "$DISPLAY" ]; then
    echo "No DISPLAY set, falling back to Xvfb..."
    if ! pgrep -x Xvfb > /dev/null; then
        Xvfb :99 -screen 0 1024x768x24 &
        sleep 1
    fi
    export DISPLAY=:99
fi

# ── Environment (works via SSH non-login shell) ──
source /opt/ros/humble/setup.bash
source "$GAZEBO_WS/install/setup.bash"
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_RESOURCE_PATH="$MODEL_DIR${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export ROS_DOMAIN_ID=1
export GAZEBO_WS

# ── Process xacro → URDF, add world fixed joint ──
export GAZEBO_WS
source "$GAZEBO_WS/install/setup.bash"
python3 "$GAZEBO_WS/scripts/prepare_urdf.py"

echo "=== Environment ==="
echo "  HEADLESS=$HEADLESS"
echo "  DISPLAY=$DISPLAY"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH"
echo ""
# ── Start Ignition Gazebo 6 ──
if [ "$HEADLESS" = "True" ] || [ "$HEADLESS" = "true" ]; then
    echo "=== Starting Ignition Gazebo 6 (server-only, headless) ==="
    ign gazebo -r -v 4 -s empty.sdf &
else
    echo "=== Starting Ignition Gazebo 6 (with GUI) ==="
    ign gazebo -r -v 4 empty.sdf &
fi
GZ_PID=$!
sleep 8

# ── Start robot_state_publisher ──
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p use_sim_time:=true \
    -p "robot_description:=$(cat /tmp/rebotarm_description.urdf)" &
sleep 2

# ── Spawn model via ign service (URDF with world fixed joint) ──
SDF=$(cat /tmp/rebotarm_description.urdf)
ign service -s /world/empty/create \
    --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean \
    --timeout 5000 \
    -r "sdf: '$SDF' name: 'rebotarm' allow_renaming: true"
sleep 3

# ── Load ros2 controllers ──
ros2 run controller_manager spawner joint_state_broadcaster &
ros2 run controller_manager spawner arm_controller &
sleep 3

# ── Start domain bridge (Jetson domain 0 → Ubuntu domain 1) ──
echo "=== Starting domain_bridge (Jetson /joint_states → /leader/joint_states) ==="
ros2 run joint_state_relay domain_bridge &
sleep 2

# ── Start relay node (leader → follower trajectory via action) ──
RELAY_CFG="$GAZEBO_WS/install/joint_state_relay/share/joint_state_relay/config/relay.yaml"
echo "=== Starting relay_node (gazebo mode, action client) ==="
ros2 run joint_state_relay relay_node --ros-args -p "config_path:=$RELAY_CFG" &

echo ""
echo "=== Gazebo simulation running ==="
echo "  - Gazebo:     ign gazebo (PID $GZ_PID)"
echo "  - Controllers: arm_controller + joint_state_broadcaster"
echo "  - Bridge:     domain_bridge (domain 0→1)"
echo "  - Relay:      relay_node (gazebo mode)"
echo ""
echo "  Move the leader arm to control the simulated follower."
echo "  Run: bash scripts/debug_gazebo_data_flow.sh  to diagnose."
wait $GZ_PID
