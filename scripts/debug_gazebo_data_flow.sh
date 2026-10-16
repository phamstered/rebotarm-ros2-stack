#!/bin/bash
# Gazebo 仿真数据流诊断脚本
# 检查: Jetson主臂 → domain_bridge → relay → Gazebo从臂 控制器
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
info() { echo -e "       $1"; }

echo "=========================================="
echo " Gazebo 仿真数据流诊断"
echo "=========================================="
echo ""

ERRORS=0

# ─── 1. 进程检查 ───
echo "── 1. 进程检查 ──"

check_proc() {
    local name="$1" pattern="$2"
    local pid
    pid=$(pgrep -f "$pattern" 2>/dev/null | head -1 || true)
    if [ -n "$pid" ]; then
        pass "$name 运行中 (PID $pid)"
    else
        # fallback: check by full command line
        pid=$(ps aux | grep -v grep | grep "$pattern" | awk '{print $2}' | head -1 || true)
        if [ -n "$pid" ]; then
            pass "$name 运行中 (PID $pid)"
        else
            fail "$name 未运行"
            ERRORS=$((ERRORS+1))
        fi
    fi
}

check_proc "ign gazebo" "ign gazebo"
check_proc "robot_state_publisher" "robot_state_publisher"
check_proc "arm_controller spawner" "spawner.*arm_controller"
check_proc "joint_state_broadcaster" "spawner.*joint_state_broadcaster"
check_proc "domain_bridge" "domain_bridge"
check_proc "relay_node" "relay_node.*relay_node"

echo ""

# ─── 2. 控制器状态 ───
echo "── 2. 控制器状态 ──"

source /opt/ros/humble/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=1

CTRL_LIST=$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>/dev/null || true)

if echo "$CTRL_LIST" | grep -q "arm_controller.*active"; then
    pass "arm_controller 状态: active"
else
    fail "arm_controller 未激活"
    ERRORS=$((ERRORS+1))
fi

if echo "$CTRL_LIST" | grep -q "joint_state_broadcaster.*active"; then
    pass "joint_state_broadcaster 状态: active"
else
    fail "joint_state_broadcaster 未激活"
    ERRORS=$((ERRORS+1))
fi

echo ""

# ─── 3. Topic 检查 (domain 1) ───
echo "── 3. Topic 检查 (ROS_DOMAIN_ID=1) ──"

TOPICS=$(ros2 topic list 2>/dev/null)

check_topic() {
    local name="$1"
    if echo "$TOPICS" | grep -q "^${name}$"; then
        pass "Topic 存在: $name"
    else
        fail "Topic 缺失: $name"
        ERRORS=$((ERRORS+1))
    fi
}

check_topic "/leader/joint_states"
check_topic "/joint_states"
check_topic "/arm_controller/joint_trajectory"

echo ""

# ─── 4. 频率检查 ───
echo "── 4. 数据流频率检查 ──"

check_hz() {
    local topic="$1" label="$2" timeout="${3:-4}"
    local hz
    hz=$(timeout "$timeout" ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | tail -1 | awk '{print $3}' || true)
    if [ -n "$hz" ]; then
        local hz_int=${hz%.*}
        if [ "$hz_int" -gt 0 ] 2>/dev/null; then
            pass "$label: ${hz} Hz"
        else
            warn "$label: 频率异常 (${hz} Hz)"
        fi
    else
        fail "$label: 无数据"
        ERRORS=$((ERRORS+1))
    fi
}

check_hz "/leader/joint_states" "Jetson→Ubuntu 桥接"
check_hz "/joint_states" "Gazebo 从臂 joint_states"
# relay 使用 action 接口，topic 可能无数据，改为检查 action
ACTION_GOAL=$(timeout 3 ros2 action info /arm_controller/follow_joint_trajectory 2>/dev/null || true)
if echo "$ACTION_GOAL" | grep -q "num_goals.*[1-9]"; then
    pass "Relay action goal: 活动目标存在"
else
    check_hz "/arm_controller/joint_trajectory" "Relay 轨迹输出" 3
fi

echo ""

# ─── 5. 领导臂关节数据 ───
echo "── 5. Jetson 主臂关节数据 (/leader/joint_states) ──"

LEADER_DATA=$(timeout 3 ros2 topic echo /leader/joint_states --once 2>/dev/null || true)
if [ -n "$LEADER_DATA" ]; then
    LEADER_POS=$(echo "$LEADER_DATA" | grep -A7 'position:' | tail -6 | tr -d ' -' | tr '\n' ' ')
    pass "收到主臂数据"
    info "位置: $LEADER_POS"

    # 检查是否有非零值
    HAS_NONZERO=false
    for val in $LEADER_POS; do
        val_clean=$(echo "$val" | sed 's/[eE].*//')
        if [ "$val_clean" != "0.000000" ] && [ "$val_clean" != "0.0" ] && [ -n "$val_clean" ]; then
            HAS_NONZERO=true
            break
        fi
    done
    if $HAS_NONZERO; then
        pass "主臂关节有非零值 (已移动)"
    else
        warn "主臂关节全为零 — 请移动主臂后重试"
    fi
else
    fail "无法获取主臂数据"
    ERRORS=$((ERRORS+1))
fi

echo ""

# ─── 6. 从臂关节数据 ───
echo "── 6. Gazebo 从臂关节数据 (/joint_states) ──"

FOLLOWER_DATA=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null || true)
if [ -n "$FOLLOWER_DATA" ]; then
    FOLLOWER_POS=$(echo "$FOLLOWER_DATA" | grep -A7 'position:' | tail -6 | tr -d ' -' | tr '\n' ' ')
    pass "收到从臂数据"
    info "位置: $FOLLOWER_POS"
else
    fail "无法获取从臂数据"
    ERRORS=$((ERRORS+1))
fi

echo ""

# ─── 7. Relay 配置检查 ───
echo "── 7. Relay 配置检查 ──"

RELAY_CFG="/home/jialiang/Project/rebotarm-ros2-stack/install/joint_state_relay/share/joint_state_relay/config/relay.yaml"
if [ -f "$RELAY_CFG" ]; then
    MODE=$(grep '^mode:' "$RELAY_CFG" | awk '{print $2}' | tr -d '"')
    LEADER_TOPIC=$(grep '^leader_topic:' "$RELAY_CFG" | awk '{print $2}' | tr -d '"')
    if [ "$MODE" = "gazebo" ]; then
        pass "Relay 模式: gazebo"
    else
        fail "Relay 模式: $MODE (应为 gazebo)"
        ERRORS=$((ERRORS+1))
    fi
    info "监听 topic: $LEADER_TOPIC"
else
    fail "Relay 配置文件不存在: $RELAY_CFG"
    ERRORS=$((ERRORS+1))
fi

echo ""

# ─── 8. Action Server 检查 ───
echo "── 8. Action Server 检查 ──"

ACTIONS=$(ros2 action list 2>/dev/null)
if echo "$ACTIONS" | grep -q "/arm_controller/follow_joint_trajectory"; then
    pass "Action: /arm_controller/follow_joint_trajectory 存在"
else
    fail "Action: /arm_controller/follow_joint_trajectory 缺失"
    ERRORS=$((ERRORS+1))
fi

echo ""

# ─── 9. 轨迹内容检查 ───
echo "── 9. 轨迹内容检查 ──"

# 先检查 action 是否有 goal
ACTION_INFO=$(timeout 3 ros2 action info /arm_controller/follow_joint_trajectory 2>/dev/null || true)
if echo "$ACTION_INFO" | grep -q "num_goals.*[1-9]"; then
    pass "Relay 正在通过 action 发送轨迹"
else
    # fallback: 检查 topic
    TRAJ_DATA=$(timeout 3 ros2 topic echo /arm_controller/joint_trajectory --once 2>/dev/null || true)
    if [ -n "$TRAJ_DATA" ]; then
        TRAJ_POS=$(echo "$TRAJ_DATA" | grep -A7 'positions:' | head -7 | tail -6 | tr -d ' -' | tr '\n' ' ')
        pass "Relay 正在通过 topic 发送轨迹"
        info "轨迹位置: $TRAJ_POS"
    else
        fail "Relay 未发送轨迹数据 (action 和 topic 均无)"
        ERRORS=$((ERRORS+1))
    fi
fi

echo ""

# ─── 10. 时间戳同步检查 ───
echo "── 10. 时间同步检查 ──"

SIM_TIME=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null | grep 'sec:' | head -1 | awk '{print $2}' || true)
if [ -n "$SIM_TIME" ]; then
    pass "仿真时钟: sec=$SIM_TIME"
else
    warn "未收到 /clock (Gazebo 可能未桥接时钟)"
fi

echo ""

# ─── 总结 ───
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}所有检查通过！${NC}"
    echo "如果从臂仍未移动，请移动主臂后等待 1-2 秒。"
else
    echo -e "${RED}发现 $ERRORS 个问题${NC}"
    echo ""
    echo "常见修复:"
    echo "  - relay 未运行: source install/setup.bash && ros2 run joint_state_relay relay_node --ros-args -p config_path:=install/joint_state_relay/share/joint_state_relay/config/relay.yaml"
    echo "  - domain_bridge 未运行: source install/setup.bash && ros2 run joint_state_relay domain_bridge"
    echo "  - relay 模式错误: 编辑 relay.yaml 将 mode 改为 gazebo"
    echo "  - 控制器未激活: ros2 run controller_manager spawner arm_controller"
fi
echo "=========================================="
