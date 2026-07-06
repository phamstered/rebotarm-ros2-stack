#!/bin/bash
# ============================================================================
# 总入口脚本
# 用途: 运行完整的诊断流程
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

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 默认参数
DOMAIN_ID=0
SKIP_INTERACTIVE=false
OUTPUT_DIR="diagnostics_$(date +%Y%m%d_%H%M%S)"

# 帮助信息
show_help() {
    echo -e "${BLUE}用法${NC}: $0 [选项]"
    echo ""
    echo -e "${BLUE}选项${NC}:"
    echo "  --domain ID              ROS_DOMAIN_ID，默认: 0"
    echo "  --skip-interactive       跳过交互式测试"
    echo "  -o, --output DIR         输出目录"
    echo "  -h, --help               显示帮助"
    echo ""
    echo -e "${BLUE}示例${NC}:"
    echo "  $0                           # 运行完整诊断"
    echo "  $0 --skip-interactive        # 跳过交互式步骤"
    echo "  $0 -o my_diagnostics         # 指定输出目录"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --domain)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --skip-interactive)
            SKIP_INTERACTIVE=true
            shift
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
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

# 设置 ROS 环境
export ROS_DOMAIN_ID=${DOMAIN_ID}

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Joint4 高频振动完整诊断${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}诊断参数:${NC}"
echo "  ROS_DOMAIN_ID: ${DOMAIN_ID}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "  跳过交互: ${SKIP_INTERACTIVE}"
echo ""

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

# ============================================================================
# 诊断步骤
# ============================================================================

# 步骤 1: 环境检查
step1_environment_check() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 1/7: 环境检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 检查 ROS2
    echo -e "${BLUE}检查 ROS2 环境...${NC}"
    if command -v ros2 &> /dev/null; then
        echo -e "  ${GREEN}✅${NC} ros2 命令可用"
        source /opt/ros/humble/setup.bash 2>/dev/null || true
    else
        echo -e "  ${RED}❌${NC} ros2 命令不可用"
        exit 1
    fi

    # 检查 Python
    echo -e "${BLUE}检查 Python 环境...${NC}"
    if command -v python3 &> /dev/null; then
        echo -e "  ${GREEN}✅${NC} python3 可用: $(python3 --version)"
    else
        echo -e "  ${RED}❌${NC} python3 不可用"
        exit 1
    fi

    # 检查 numpy
    if python3 -c "import numpy" 2>/dev/null; then
        echo -e "  ${GREEN}✅${NC} numpy 可用"
    else
        echo -e "  ${YELLOW}⚠️${NC}  numpy 不可用，部分分析功能受限"
    fi

    # 检查 matplotlib
    if python3 -c "import matplotlib" 2>/dev/null; then
        echo -e "  ${GREEN}✅${NC} matplotlib 可用"
    else
        echo -e "  ${YELLOW}⚠️${NC}  matplotlib 不可用，图表功能受限"
    fi

    echo ""

    # 保存环境信息
    cat > "${OUTPUT_DIR}/environment.txt" << EOF
诊断时间: $(date)
主机名: $(hostname)
ROS2 版本: $(ros2 --version 2>&1 || echo "未知")
Python 版本: $(python3 --version)
ROS_DOMAIN_ID: ${DOMAIN_ID}
EOF

    echo -e "环境信息已保存: ${OUTPUT_DIR}/environment.txt"
    echo ""
}

# 步骤 2: 通信检查
step2_communication_check() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 2/7: 通信检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    echo "检查 DDS 通信质量..."
    echo ""

    # 检查 topic 列表
    echo -e "${BLUE}可用 Topics:${NC}"
    timeout 5 ros2 topic list 2>/dev/null | head -20 || echo "  无法获取 topic 列表"
    echo ""

    # 检查关键 topic
    echo -e "${BLUE}关键 Topic 信息:${NC}"
    for topic in /joint_states /joint_commands; do
        echo ""
        echo "  $topic:"
        timeout 3 ros2 topic info $topic 2>/dev/null | sed 's/^/    /' || echo "    无法获取信息"
    done
    echo ""

    # 保存通信信息
    timeout 10 ros2 topic list -t > "${OUTPUT_DIR}/topics.txt" 2>/dev/null || true
    echo -e "Topic 列表已保存: ${OUTPUT_DIR}/topics.txt"
    echo ""
}

# 步骤 3: 数据采集
step3_data_capture() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 3/7: 数据采集${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    if [ "$SKIP_INTERACTIVE" = true ]; then
        echo -e "${YELLOW}跳过交互式数据采集${NC}"
        echo ""
        return
    fi

    echo -e "${YELLOW}此步骤将录制关节数据用于分析${NC}"
    echo ""
    echo -e "${BLUE}操作指南:${NC}"
    echo "  1. 确保主臂和从臂都已启动"
    echo "  2. 操作主臂到 joint4 振动位置"
    echo "  3. 保持在振动位置"
    echo ""

    read -p "按 Enter 开始采集（10秒）..." -r

    echo ""
    echo "采集数据中..."

    # 使用 ros2 bag 录制
    timeout 10 ros2 bag record \
        -o "${OUTPUT_DIR}/joint_data" \
        --max-bag-size 0 \
        /joint_states /joint_commands 2>&1 &
    BAG_PID=$!

    # 等待完成
    sleep 10
    kill -INT ${BAG_PID} 2>/dev/null || true
    wait ${BAG_PID} 2>/dev/null || true

    echo ""
    echo -e "${GREEN}数据采集完成${NC}"
    echo ""
}

# 步骤 4: 实时监控
step4_realtime_monitor() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 4/7: 实时监控${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    if [ "$SKIP_INTERACTIVE" = true ]; then
        echo -e "${YELLOW}跳过实时监控${NC}"
        echo ""
        return
    fi

    echo -e "${YELLOW}此步骤将实时监控 joint4 状态${NC}"
    echo ""
    echo -e "${BLUE}操作指南:${NC}"
    echo "  1. 操作主臂观察 joint4 行为"
    echo "  2. 注意抖动报警"
    echo ""

    read -p "按 Enter 开始监控（15秒）..." -r

    echo ""

    # 运行监控脚本
    python3 "${SCRIPT_DIR}/06_monitor_joint4.py" \
        -d 15 \
        --domain ${DOMAIN_ID} \
        2>&1 | tee "${OUTPUT_DIR}/realtime_monitor.log"

    echo ""
}

# 步骤 5: 电机参数检查
step5_motor_params() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 5/7: 电机参数检查${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    echo "检查电机配置..."
    echo ""

    # 运行电机参数检查
    "${SCRIPT_DIR}/07_check_motor_params.sh" 2>&1 | tee "${OUTPUT_DIR}/motor_params.log"

    echo ""
}

# 步骤 6: 数据分析
step6_data_analysis() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 6/7: 数据分析${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 检查是否有录制数据
    if [ -d "${OUTPUT_DIR}/joint_data" ]; then
        echo "分析录制的数据..."
        echo ""

        # 运行 bag 分析
        python3 "${SCRIPT_DIR}/02_analyze_bag.py" \
            "${OUTPUT_DIR}/joint_data" \
            --output "${OUTPUT_DIR}" \
            2>&1 | tee "${OUTPUT_DIR}/bag_analysis.log"

        echo ""
    else
        echo -e "${YELLOW}未找到录制数据，跳过 bag 分析${NC}"
        echo ""
    fi

    # 频谱分析
    echo "执行频谱分析..."
    python3 "${SCRIPT_DIR}/08_analyze_frequency.py" \
        --test \
        --output "${OUTPUT_DIR}" \
        2>&1 | tee "${OUTPUT_DIR}/frequency_analysis.log"

    echo ""
}

# 步骤 7: 生成报告
step7_generate_report() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  步骤 7/7: 生成报告${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""

    # 创建汇总报告
    REPORT_FILE="${OUTPUT_DIR}/diagnostic_report.txt"

    cat > "${REPORT_FILE}" << EOF
========================================
Joint4 高频振动诊断报告
========================================

诊断时间: $(date)
主机名: $(hostname)
ROS_DOMAIN_ID: ${DOMAIN_ID}

========================================
诊断步骤
========================================

1. 环境检查
   - ROS2 环境: 正常
   - Python 环境: 正常

2. 通信检查
   - 详见: topics.txt

3. 数据采集
   - 详见: joint_data/ 目录

4. 实时监控
   - 详见: realtime_monitor.log

5. 电机参数
   - 详见: motor_params.log

6. 数据分析
   - 详见: bag_analysis.log
   - 详见: frequency_analysis.log

========================================
诊断结果
========================================

EOF

    # 合并所有分析结果
    if [ -f "${OUTPUT_DIR}/analysis_report.txt" ]; then
        cat "${OUTPUT_DIR}/analysis_report.txt" >> "${REPORT_FILE}"
    fi

    if [ -f "${OUTPUT_DIR}/frequency_report.txt" ]; then
        echo "" >> "${REPORT_FILE}"
        cat "${OUTPUT_DIR}/frequency_report.txt" >> "${REPORT_FILE}"
    fi

    # 添加建议
    cat >> "${REPORT_FILE}" << EOF

========================================
下一步建议
========================================

根据诊断结果，建议采取以下措施：

1. 如果检测到高频振动 (50-100Hz):
   - 降低 Kd 增益 30-50%
   - 增加速度滤波

2. 如果检测到低频振荡 (<10Hz):
   - 检查机械结构
   - 调整重力补偿

3. 如果通信不稳定:
   - 检查 DDS 配置
   - 检查网络连接

4. 如果电机参数异常:
   - 调整 MIT 控制参数
   - 检查力矩限制

========================================
详细数据
========================================

所有诊断数据已保存到: ${OUTPUT_DIR}/

主要文件:
- diagnostic_report.txt (本报告)
- environment.txt (环境信息)
- topics.txt (Topic 列表)
- realtime_monitor.log (实时监控)
- motor_params.log (电机参数)
- bag_analysis.log (数据分析)
- frequency_analysis.log (频谱分析)
- analysis_report.txt (详细分析)
- frequency_report.txt (频谱报告)

========================================
EOF

    echo -e "${GREEN}诊断报告已生成${NC}"
    echo -e "报告位置: ${BLUE}${REPORT_FILE}${NC}"
    echo ""
}

# ============================================================================
# 主流程
# ============================================================================

main() {
    echo -e "${GREEN}开始完整诊断流程...${NC}"
    echo ""

    step1_environment_check
    step2_communication_check
    step3_data_capture
    step4_realtime_monitor
    step5_motor_params
    step6_data_analysis
    step7_generate_report

    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  诊断完成${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "输出目录: ${BLUE}${OUTPUT_DIR}/${NC}"
    echo -e "诊断报告: ${BLUE}${OUTPUT_DIR}/diagnostic_report.txt${NC}"
    echo ""
    echo -e "${YELLOW}查看报告:${NC}"
    echo "  cat ${OUTPUT_DIR}/diagnostic_report.txt"
    echo ""
    echo -e "${YELLOW}下一步:${NC}"
    echo "  1. 查看诊断报告"
    echo "  2. 根据建议调整参数"
    echo "  3. 重新测试验证"
    echo ""
}

main
