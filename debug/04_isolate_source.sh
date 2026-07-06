#!/bin/bash
# ============================================================================
# 步骤4: 隔离问题源
# 用途: 通过引导式测试，判断问题在数据层还是控制层
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
OUTPUT_DIR="isolation_test_$(date +%Y%m%d_%H%M%S)"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  问题源隔离测试${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}本测试将通过一系列步骤帮助定位振动来源:${NC}"
echo "  1. 检查主臂原始数据"
echo "  2. 检查 relay 转换后数据"
echo "  3. 断开从臂测试"
echo "  4. 对比分析"
echo ""

# 设置 ROS 环境
export ROS_DOMAIN_ID=${DOMAIN_ID}

# 创建输出目录
mkdir -p ${OUTPUT_DIR}

# ============================================================================
# 测试 1: 主臂原始数据
# ============================================================================

test_leader_raw() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  测试 1: 主臂原始数据${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}目的${NC}: 检查主臂发送的数据是否有抖动"
    echo ""
    echo -e "${BLUE}操作步骤:${NC}"
    echo "  1. 确保主臂已启动"
    echo "  2. 操作主臂到 joint4 振动位置"
    echo "  3. 保持 5 秒"
    echo ""
    echo -e "${YELLOW}按 Enter 开始采集...${NC}"
    read -r

    echo -e "${BLUE}采集主臂数据 (5秒)...${NC}"
    timeout 5 ros2 topic echo /joint_states --field-name position \
        > ${OUTPUT_DIR}/leader_raw.txt 2>&1 || true

    DATA_LINES=$(wc -l < ${OUTPUT_DIR}/leader_raw.txt)
    echo -e "采集完成: ${GREEN}${DATA_LINES}${NC} 行数据"
    echo ""

    # 快速分析抖动
    if [ ${DATA_LINES} -gt 10 ]; then
        echo -e "${BLUE}快速分析...${NC}"
        python3 - << 'EOF'
import re
import sys

data_file = "${OUTPUT_DIR}/leader_raw.txt"

# 读取数据
positions = []
with open(data_file) as f:
    for line in f:
        # 解析 position 数组
        match = re.search(r'\[([^\]]+)\]', line)
        if match:
            try:
                pos = [float(x.strip()) for x in match.group(1).split(',')]
                if len(pos) >= 4:
                    positions.append(pos[3])  # joint4
            except:
                pass

if len(positions) > 10:
    # 计算变化率
    diffs = [abs(positions[i] - positions[i-1]) for i in range(1, len(positions))]
    avg_diff = sum(diffs) / len(diffs)
    max_diff = max(diffs)

    print(f"数据点数: {len(positions)}")
    print(f"平均变化: {avg_diff:.6f}")
    print(f"最大变化: {max_diff:.6f}")

    if max_diff > 0.01:  # 超过 0.01 弧度
        print("⚠️  主臂数据存在明显抖动")
    else:
        print("✅ 主臂数据平滑")
else:
    print("数据不足，无法分析")
EOF
    fi
    echo ""
}

# ============================================================================
# 测试 2: Relay 输出数据
# ============================================================================

test_relay_output() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  测试 2: Relay 输出数据${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}目的${NC}: 检查 relay 转换后的数据"
    echo ""
    echo -e "${BLUE}操作步骤:${NC}"
    echo "  1. 确保 relay_node 已启动"
    echo "  2. 操作主臂到 joint4 振动位置"
    echo "  3. 保持 5 秒"
    echo ""
    echo -e "${YELLOW}按 Enter 开始采集...${NC}"
    read -r

    echo -e "${BLUE}采集 relay 数据 (5秒)...${NC}"
    timeout 5 ros2 topic echo /joint_commands --field-name position \
        > ${OUTPUT_DIR}/relay_output.txt 2>&1 || true

    DATA_LINES=$(wc -l < ${OUTPUT_DIR}/relay_output.txt)
    echo -e "采集完成: ${GREEN}${DATA_LINES}${NC} 行数据"
    echo ""
}

# ============================================================================
# 测试 3: 对比分析
# ============================================================================

compare_data() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  测试 3: 数据对比分析${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}目的${NC}: 对比主臂和 relay 数据，判断 relay 是否引入抖动"
    echo ""

    python3 - << 'EOF'
import re
import sys

def parse_positions(filename):
    positions = []
    try:
        with open(filename) as f:
            for line in f:
                match = re.search(r'\[([^\]]+)\]', line)
                if match:
                    try:
                        pos = [float(x.strip()) for x in match.group(1).split(',')]
                        if len(pos) >= 4:
                            positions.append(pos[3])  # joint4
                    except:
                        pass
    except:
        pass
    return positions

leader_pos = parse_positions("${OUTPUT_DIR}/leader_raw.txt")
relay_pos = parse_positions("${OUTPUT_DIR}/relay_output.txt")

print(f"主臂数据点: {len(leader_pos)}")
print(f"Relay 数据点: {len(relay_pos)}")
print("")

if len(leader_pos) > 10 and len(relay_pos) > 10:
    # 计算抖动指标
    def calc_jitter(pos):
        diffs = [abs(pos[i] - pos[i-1]) for i in range(1, len(pos))]
        return sum(diffs) / len(diffs), max(diffs)

    leader_avg, leader_max = calc_jitter(leader_pos)
    relay_avg, relay_max = calc_jitter(relay_pos)

    print("抖动对比:")
    print(f"  主臂: 平均={leader_avg:.6f}, 最大={leader_max:.6f}")
    print(f"  Relay: 平均={relay_avg:.6f}, 最大={relay_max:.6f}")
    print("")

    # 判断
    ratio = relay_max / leader_max if leader_max > 0 else 1

    if ratio > 2:
        print("⚠️  Relay 输出抖动明显大于主臂输入")
        print("  可能原因:")
        print("    - Relay 转换逻辑有问题")
        print("    - 数据类型转换精度丢失")
        print("    - 单位转换错误")
    elif ratio < 0.5:
        print("ℹ️  Relay 输出比主臂输入更平滑")
        print("  这是正常的，relay 可能有滤波")
    else:
        print("✅ 两者抖动程度相近")
        print("  问题可能在从臂驱动层")
else:
    print("数据不足，无法对比")
EOF
    echo ""
}

# ============================================================================
# 测试 4: 断开从臂测试
# ============================================================================

test_without_follower() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  测试 4: 断开从臂测试${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "${YELLOW}目的${NC}: 确认问题是否在从臂控制层"
    echo ""
    echo -e "${RED}重要${NC}: 此测试需要停止从臂驱动！"
    echo ""
    echo -e "${BLUE}操作步骤:${NC}"
    echo "  1. 停止从臂驱动 (Ctrl+C 或 kill)"
    echo "  2. 保持主臂和 relay 运行"
    echo "  3. 操作主臂观察是否还有振动感"
    echo ""
    echo -e "${YELLOW}是否已停止从臂驱动？(y/n)${NC}"
    read -r answer

    if [[ "$answer" != "y" && "$answer" != "Y" ]]; then
        echo -e "${YELLOW}跳过此测试${NC}"
        return
    fi

    echo ""
    echo -e "${BLUE}操作主臂到振动位置，观察是否还有振动感${NC}"
    echo ""
    echo -e "${YELLOW}是否感觉到振动？(y/n)${NC}"
    read -r has_vibration

    echo ""
    if [[ "$has_vibration" == "y" || "$has_vibration" == "Y" ]]; then
        echo -e "${RED}⚠️  断开从臂后仍有振动感${NC}"
        echo "  问题可能在:"
        echo "    - 主臂机械结构"
        echo "    - 主臂电机驱动"
        echo "    - 你的手臂疲劳（开玩笑的）"
    else
        echo -e "${GREEN}✅ 断开从臂后无振动${NC}"
        echo "  问题确认在从臂控制层"
        echo ""
        echo -e "${YELLOW}下一步检查:${NC}"
        echo "  1. 从臂电机参数 (Kp/Kd)"
        echo "  2. 从臂机械结构"
        echo "  3. 运行: ./debug/07_check_motor_params.sh"
    fi
    echo ""
}

# ============================================================================
# 总结
# ============================================================================

generate_summary() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  隔离测试总结${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    echo -e "测试数据已保存到: ${BLUE}${OUTPUT_DIR}/${NC}"
    echo ""
    echo -e "${YELLOW}根据测试结果，参考以下排查方向:${NC}"
    echo ""
    echo -e "1. ${BLUE}主臂数据有抖动${NC}"
    echo "   → 检查主臂机械结构、电机驱动"
    echo ""
    echo -e "2. ${BLUE}Relay 引入抖动${NC}"
    echo "   → 检查 relay_node 转换逻辑"
    echo "   → 查看: src/joint_state_relay/config/relay_real.yaml"
    echo ""
    echo -e "3. ${BLUE}从臂控制层问题${NC}"
    echo "   → 调整 MIT 控制参数 (Kp/Kd)"
    echo "   → 检查从臂机械结构"
    echo "   → 运行: ./debug/07_check_motor_params.sh"
    echo ""
    echo -e "4. ${BLUE}通信问题${NC}"
    echo "   → 检查 DDS 配置"
    echo "   → 运行: ./debug/03_check_communication.sh"
    echo ""
}

# ============================================================================
# 主流程
# ============================================================================

main() {
    test_leader_raw
    test_relay_output
    compare_data
    test_without_follower
    generate_summary
}

main
