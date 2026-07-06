#!/bin/bash
# ============================================================================
# 步骤3: 检查通信质量
# 用途: 检查 DDS 通信的频率、延迟和稳定性
# 运行位置: Jetson (与从臂通信) 或 Ubuntu (与 Gazebo 通信)
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 默认参数
DURATION=10
DOMAIN_ID=0
CHECK_TOPICS=("/joint_states" "/joint_commands")

# 帮助信息
show_help() {
    echo -e "${BLUE}用法${NC}: $0 [选项]"
    echo ""
    echo -e "${BLUE}选项${NC}:"
    echo "  -d, --duration SECONDS   检查时长（秒），默认: 10"
    echo "  --domain ID              ROS_DOMAIN_ID，默认: 0"
    echo "  --topic TOPIC            指定检查的 topic（可多次使用）"
    echo "  -h, --help               显示帮助"
    echo ""
    echo -e "${BLUE}示例${NC}:"
    echo "  $0                           # 检查默认 topics"
    echo "  $0 -d 30                     # 检查 30 秒"
    echo "  $0 --topic /joint_states     # 只检查特定 topic"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        --domain)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --topic)
            CHECK_TOPICS=("$2")
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误${NC}: 未知参数 $1"
            show_help
            exit 1
            ;;
    esac
done

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  通信质量检查${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}参数:${NC}"
echo "  检查时长: ${DURATION}秒"
echo "  ROS_DOMAIN_ID: ${DOMAIN_ID}"
echo "  检查 Topics: ${CHECK_TOPICS[*]}"
echo ""

# 设置 ROS 环境
export ROS_DOMAIN_ID=${DOMAIN_ID}

# ============================================================================
# 检查函数
# ============================================================================

check_topic_hz() {
    local topic=$1
    echo -e "${BLUE}检查 ${topic} 频率...${NC}"

    # 运行 ros2 topic hz
    timeout ${DURATION} ros2 topic hz ${topic} 2>&1 | tail -1
    echo ""
}

check_topic_delay() {
    local topic=$1
    echo -e "${BLUE}检查 ${topic} 延迟...${NC}"

    # 运行 ros2 topic delay
    timeout ${DURATION} ros2 topic delay ${topic} 2>&1 | tail -5
    echo ""
}

check_topic_bw() {
    local topic=$1
    echo -e "${BLUE}检查 ${topic} 带宽...${NC}"

    # 运行 ros2 topic bw
    timeout ${DURATION} ros2 topic bw ${topic} 2>&1 | tail -3
    echo ""
}

check_topic_info() {
    local topic=$1
    echo -e "${BLUE}检查 ${topic} 信息...${NC}"

    ros2 topic info ${topic} --verbose 2>&1
    echo ""
}

# ============================================================================
# 执行检查
# ============================================================================

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Topic 信息${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

for topic in "${CHECK_TOPICS[@]}"; do
    check_topic_info ${topic}
done

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  频率检查 (${DURATION}秒)${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

for topic in "${CHECK_TOPICS[@]}"; do
    check_topic_hz ${topic}
done

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  延迟检查${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

for topic in "${CHECK_TOPICS[@]}"; do
    check_topic_delay ${topic}
done

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  带宽检查${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

for topic in "${CHECK_TOPICS[@]}"; do
    check_topic_bw ${topic}
done

# ============================================================================
# DDS 诊断
# ============================================================================

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  DDS 诊断${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 检查 DDS 实现
echo -e "${BLUE}DDS 实现:${NC}"
echo ${RMW_IMPLEMENTATION:-"未设置（使用默认）"}
echo ""

# 检查网络接口
echo -e "${BLUE}网络接口:${NC}"
ifconfig 2>/dev/null | grep -E "^[a-z]|inet " | head -20 || echo "无法获取网络信息"
echo ""

# 检查多播路由
echo -e "${BLUE}多播路由:${NC}"
ip route show multicast 2>/dev/null || echo "无法获取多播路由信息"
echo ""

# ============================================================================
# 诊断结果
# ============================================================================

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  诊断建议${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 获取频率信息（静默）
for topic in "${CHECK_TOPICS[@]}"; do
    HZ_OUTPUT=$(timeout 3 ros2 topic hz ${topic} 2>&1 | grep "average rate" || echo "")

    if [ -n "$HZ_OUTPUT" ]; then
        HZ=$(echo "$HZ_OUTPUT" | grep -oP 'average rate: \K[0-9.]+')

        if [ -n "$HZ" ]; then
            # 判断频率是否正常
            if (( $(echo "$HZ < 50" | bc -l) )); then
                echo -e "${RED}⚠️  ${topic} 频率偏低: ${HZ} Hz${NC}"
                echo "  可能原因:"
                echo "    - 发布节点负载过高"
                echo "    - DDS 通信拥塞"
                echo "    - 网络延迟"
            elif (( $(echo "$HZ > 1000" | bc -l) )); then
                echo -e "${YELLOW}⚠️  ${topic} 频率异常高: ${HZ} Hz${NC}"
                echo "  可能原因:"
                echo "    - 采样率配置错误"
                echo "    - 数据重复发送"
            else
                echo -e "${GREEN}✅ ${topic} 频率正常: ${HZ} Hz${NC}"
            fi
        fi
    fi
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  检查完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo "  如果频率不稳定，运行: ./debug/04_isolate_source.sh"
echo "  如果需要对比数据，运行: python3 debug/05_compare_topics.py"
