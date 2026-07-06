#!/bin/bash
# ============================================================================
# 步骤7: 检查电机参数
# 用途: 检查 joint4 电机配置，包括控制参数和状态
# 运行位置: Jetson
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

DOMAIN_ID=0

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  电机参数检查${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# ============================================================================
# 检查配置文件
# ============================================================================

check_config_files() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  1. 配置文件检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 查找项目根目录
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

    # 检查 relay 配置
    echo -e "${BLUE}Relay 配置:${NC}"
    RELAY_CONFIG="${PROJECT_DIR}/src/joint_state_relay/config/relay_real.yaml"
    if [ -f "$RELAY_CONFIG" ]; then
        echo "  文件: $RELAY_CONFIG"
        echo "  内容:"
        cat "$RELAY_CONFIG" | grep -A 5 -B 5 "joint4\|sign\|offset" | head -30
        echo ""
    else
        echo -e "  ${YELLOW}未找到配置文件${NC}"
    fi

    # 检查从臂驱动配置
    echo -e "${BLUE}从臂驱动配置:${NC}"
    DRIVER_CONFIG="${PROJECT_DIR}/src/rebotarm_hw_driver/config"
    if [ -d "$DRIVER_CONFIG" ]; then
        echo "  目录: $DRIVER_CONFIG"
        ls -la "$DRIVER_CONFIG" 2>/dev/null || echo "  无法访问"
    else
        echo -e "  ${YELLOW}未找到配置目录${NC}"
    fi
    echo ""
}

# ============================================================================
# 检查电机状态
# ============================================================================

check_motor_status() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  2. 电机状态检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 设置 ROS 环境
    export ROS_DOMAIN_ID=${DOMAIN_ID}

    echo -e "${BLUE}检查 joint_states topic:${NC}"
    echo ""

    # 获取最新的关节状态
    echo "采样 5 秒的关节数据..."
    echo ""

    python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys

class MotorChecker(Node):
    def __init__(self):
        super().__init__('motor_checker')

        self.joint4_data = {
            'positions': [],
            'velocities': [],
            'efforts': []
        }

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

    def callback(self, msg):
        if len(msg.position) > 3:
            self.joint4_data['positions'].append(msg.position[3])
            if len(msg.velocity) > 3:
                self.joint4_data['velocities'].append(msg.velocity[3])
            if len(msg.effort) > 3:
                self.joint4_data['efforts'].append(msg.effort[3])

rclpy.init()
checker = MotorChecker()

print("采集数据中...")
start = time.time()
while time.time() - start < 5:
    rclpy.spin_once(checker, timeout_sec=0.1)

# 分析数据
import numpy as np

if checker.joint4_data['positions']:
    pos = np.array(checker.joint4_data['positions'])
    print(f"\nJoint4 状态:")
    print(f"  位置范围: {np.degrees(np.min(pos)):.2f}° ~ {np.degrees(np.max(pos)):.2f}°")
    print(f"  位置标准差: {np.degrees(np.std(pos)):.4f}°")

    if checker.joint4_data['velocities']:
        vel = np.array(checker.joint4_data['velocities'])
        print(f"  速度范围: {np.min(vel):.3f} ~ {np.max(vel):.3f} rad/s")
        print(f"  速度标准差: {np.std(vel):.4f} rad/s")

    if checker.joint4_data['efforts']:
        eff = np.array(checker.joint4_data['efforts'])
        print(f"  力矩范围: {np.min(eff):.3f} ~ {np.max(eff):.3f}")
        print(f"  力矩标准差: {np.std(eff):.4f}")

    # 抖动检测
    if len(pos) > 10:
        pos_diff = np.diff(pos)
        jitter = np.std(pos_diff)
        print(f"\n  抖动指标: {jitter:.6f}")
        if jitter > 0.01:
            print(f"  ⚠️  存在明显抖动")
        elif jitter > 0.005:
            print(f"  ⚠️  存在轻微抖动")
        else:
            print(f"  ✅ 运行平稳")
else:
    print("未收到数据")

checker.destroy_node()
rclpy.shutdown()
EOF

    echo ""
}

# ============================================================================
# 检查 CAN 通信
# ============================================================================

check_can_bus() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  3. CAN 总线检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 检查 CAN 接口
    echo -e "${BLUE}CAN 接口状态:${NC}"
    if command -v ip &> /dev/null; then
        ip link show type can 2>/dev/null || echo "未找到 CAN 接口"
    else
        echo "ip 命令不可用"
    fi
    echo ""

    # 检查 can-utils
    echo -e "${BLUE}CAN 工具:${NC}"
    if command -v candump &> /dev/null; then
        echo "  candump: 可用"
    else
        echo "  candump: 不可用"
    fi

    if command -v cansend &> /dev/null; then
        echo "  cansend: 可用"
    else
        echo "  cansend: 不可用"
    fi
    echo ""

    # 尝试读取 CAN 数据
    echo -e "${BLUE}CAN 数据测试:${NC}"
    echo "  尝试读取 CAN 数据 (3秒)..."
    timeout 3 candump can0 2>/dev/null | head -10 || echo "  无法读取 CAN 数据"
    echo ""
}

# ============================================================================
# 检查控制参数
# ============================================================================

check_control_params() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  4. 控制参数检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    echo -e "${BLUE}MIT 控制参数:${NC}"
    echo "  检查从臂驱动节点的参数..."
    echo ""

    # 获取节点参数
    export ROS_DOMAIN_ID=${DOMAIN_ID}

    python3 - << 'EOF'
import rclpy
from rclpy.node import Node
import time

rclpy.init()

# 列出所有节点
print("ROS2 节点列表:")
nodes = rclpy.get_node_names()
for node in nodes:
    print(f"  - {node}")

print()

# 尝试获取 follower_driver 参数
try:
    # 创建临时节点获取参数
    node = rclpy.create_node('param_checker')

    # 等待参数服务可用
    time.sleep(1)

    # 列出参数
    param_names = node.get_parameter_names()
    print("可用参数:")
    for param in param_names:
        print(f"  - {param}")

except Exception as e:
    print(f"获取参数失败: {e}")

rclpy.shutdown()
EOF

    echo ""
    echo -e "${YELLOW}建议:${NC}"
    echo "  检查以下参数:"
    echo "  - Kp (位置增益): 过高会导致振荡"
    echo "  - Kd (速度增益): 过高会放大噪声"
    echo "  - 力矩限制: 可能导致饱和"
    echo ""
}

# ============================================================================
# 诊断建议
# ============================================================================

generate_recommendations() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  诊断建议${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    echo -e "${YELLOW}根据检查结果，考虑以下调整:${NC}"
    echo ""
    echo -e "1. ${BLUE}如果 Kd 过高${NC}"
    echo "   - 降低 30-50%"
    echo "   - 在 follower_driver_node.py 中修改"
    echo ""
    echo -e "2. ${BLUE}如果位置抖动明显${NC}"
    echo "   - 增加位置滤波"
    echo "   - 降低位置增益 Kp"
    echo ""
    echo -e "3. ${BLUE}如果力矩饱和${NC}"
    echo "   - 增大力矩限制"
    echo "   - 检查机械是否卡滞"
    echo ""
    echo -e "4. ${BLUE}如果 CAN 通信不稳定${NC}"
    echo "   - 检查 CAN 终端电阻"
    echo "   - 降低 CAN 波特率"
    echo ""
    echo -e "5. ${BLUE}如果编码器噪声大${NC}"
    echo "   - 增加速度滤波"
    echo "   - 检查编码器连接"
    echo ""
}

# ============================================================================
# 主流程
# ============================================================================

main() {
    check_config_files
    check_motor_status
    check_can_bus
    check_control_params
    generate_recommendations

    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  检查完成${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}下一步:${NC}"
    echo "  如果需要频谱分析，运行: python3 debug/08_analyze_frequency.py"
    echo "  如果需要完整诊断，运行: ./debug/run_diagnostics.sh"
}

main
