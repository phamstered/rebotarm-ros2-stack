#!/bin/bash
# ============================================================================
# 步骤1: 录制振动数据
# 用途: 录制关节状态和命令数据，用于后续分析
# 运行位置: Jetson
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 默认参数
DURATION=30
OUTPUT_DIR=""
DOMAIN_ID=0
TOPICS="/joint_states /joint_commands /follower/joint_states"

# 帮助信息
show_help() {
    echo -e "${BLUE}用法${NC}: $0 [选项]"
    echo ""
    echo -e "${BLUE}选项${NC}:"
    echo "  -d, --duration SECONDS   录制时长（秒），默认: 30"
    echo "  -o, --output DIR         输出目录，默认: vibration_<timestamp>"
    echo "  --domain ID              ROS_DOMAIN_ID，默认: 0"
    echo "  -h, --help               显示帮助"
    echo ""
    echo -e "${BLUE}示例${NC}:"
    echo "  $0                       # 录制 30 秒"
    echo "  $0 -d 60                 # 录制 60 秒"
    echo "  $0 -o my_test            # 指定输出目录"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --domain)
            DOMAIN_ID="$2"
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

# 设置输出目录
if [ -z "$OUTPUT_DIR" ]; then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    OUTPUT_DIR="vibration_${TIMESTAMP}"
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Joint4 振动数据录制${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}录制参数:${NC}"
echo "  时长: ${DURATION}秒"
echo "  输出: ${OUTPUT_DIR}/"
echo "  ROS_DOMAIN_ID: ${DOMAIN_ID}"
echo "  Topics: ${TOPICS}"
echo ""

# 检查环境
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误${NC}: 未找到 ros2 命令"
    exit 1
fi

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

# 保存元数据
cat > "${OUTPUT_DIR}/metadata.json" << EOF
{
    "timestamp": "$(date -Iseconds)",
    "duration": ${DURATION},
    "domain_id": ${DOMAIN_ID},
    "topics": ["/joint_states", "/joint_commands", "/follower/joint_states"],
    "hostname": "$(hostname)",
    "purpose": "Joint4 高频振动诊断"
}
EOF

echo -e "${YELLOW}准备录制...${NC}"
echo ""
echo -e "${BLUE}操作指南:${NC}"
echo "  1. 确保主臂和从臂都已启动"
echo "  2. 操作主臂到 joint4 振动位置"
echo "  3. 保持在振动位置直到录制结束"
echo ""
echo -e "${YELLOW}按 Enter 开始录制，Ctrl+C 提前结束${NC}"
read -r

# 设置 ROS 环境
export ROS_DOMAIN_ID=${DOMAIN_ID}

echo -e "${GREEN}开始录制...${NC}"
echo ""

# 使用 ros2 bag 录制
# 后台录制，捕获 Ctrl+C
ros2 bag record \
    -o "${OUTPUT_DIR}/joint_data" \
    --max-bag-size 0 \
    --max-cache-size 100000000 \
    ${TOPICS} &
BAG_PID=$!

# 等待指定时长或用户中断
sleep ${DURATION} &
SLEEP_PID=$!

# 等待任一进程结束
wait -n ${BAG_PID} ${SLEEP_PID} 2>/dev/null || true

# 停止录制
echo ""
echo -e "${YELLOW}停止录制...${NC}"
kill -INT ${BAG_PID} 2>/dev/null || true
wait ${BAG_PID} 2>/dev/null || true

# 获取 bag 文件大小
BAG_SIZE=$(du -sh "${OUTPUT_DIR}/joint_data" 2>/dev/null | cut -f1)

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  录制完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "输出目录: ${BLUE}${OUTPUT_DIR}/${NC}"
echo -e "数据大小: ${BLUE}${BAG_SIZE}${NC}"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo "  python3 debug/02_analyze_bag.py ${OUTPUT_DIR}/"
