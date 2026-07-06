# Joint4 振动诊断 - 快速参考

## 🚀 快速开始

```bash
# 在 Jetson 上运行完整诊断
./debug/run_diagnostics.sh

# 或跳过交互式步骤
./debug/run_diagnostics.sh --skip-interactive
```

## 📋 分步诊断

### 1️⃣ 录制振动数据
```bash
./debug/01_capture_vibration.sh -d 30
```

### 2️⃣ 分析录制数据
```bash
python3 debug/02_analyze_bag.py vibration_*/
```

### 3️⃣ 检查通信质量
```bash
./debug/03_check_communication.sh -d 10
```

### 4️⃣ 隔离问题源
```bash
./debug/04_isolate_source.sh
```

### 5️⃣ 对比 topic 数据
```bash
python3 debug/05_compare_topics.py -d 10
python3 debug/05_compare_topics.py --plot  # 图形模式
```

### 6️⃣ 实时监控 joint4
```bash
python3 debug/06_monitor_joint4.py -d 30
python3 debug/06_monitor_joint4.py --plot  # 图形模式
```

### 7️⃣ 检查电机参数
```bash
./debug/07_check_motor_params.sh
```

### 8️⃣ 频谱分析
```bash
python3 debug/08_analyze_frequency.py vibration_*/
python3 debug/08_analyze_frequency.py --test  # 测试模式
```

## 🔍 常见振动模式

| 频率范围 | 可能原因 | 建议操作 |
|---------|---------|---------|
| < 10 Hz | 机械间隙、重力补偿 | 检查螺丝、调整补偿 |
| 10-50 Hz | Kp 过高 | 降低位置增益 |
| 50-100 Hz | Kd 过高 | 降低速度增益 30-50% |
| > 100 Hz | 编码器噪声、通信抖动 | 检查编码器、通信 |

## 🛠️ 参数调整

### 调整 Kd (速度增益)
```bash
# 编辑从臂驱动配置
vim src/rebotarm_hw_driver/config/params.yaml

# 修改 Kd 值，例如降低 50%
# Kd: 0.5 → 0.25
```

### 调整 Kp (位置增益)
```bash
# 同样在配置文件中
# Kp: 1.0 → 0.7
```

## 📊 输出文件说明

```
diagnostics_*/
├── diagnostic_report.txt    # 汇总报告
├── environment.txt          # 环境信息
├── topics.txt              # Topic 列表
├── joint_data/             # 录制数据
├── realtime_monitor.log    # 实时监控日志
├── motor_params.log        # 电机参数
├── bag_analysis.log        # 数据分析
├── frequency_analysis.log  # 频谱分析
├── analysis_report.txt     # 详细分析报告
└── frequency_report.txt    # 频谱报告
```

## ⚠️ 安全提示

1. **调试时确保工作空间内无人**
2. **备份原始参数后再修改**
3. **每次只改一个参数**
4. **修改后重新测试验证**

## 🆘 遇到问题？

1. **脚本无法运行**
   ```bash
   # 检查权限
   ls -la debug/
   # 添加权限
   chmod +x debug/*.sh
   ```

2. **Python 模块缺失**
   ```bash
   pip install numpy matplotlib rosbags
   ```

3. **ROS2 未找到**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **CAN 总线问题**
   ```bash
   # 检查 CAN 接口
   ip link show type can
   # 启动 CAN 接口
   sudo ip link set can0 up type can bitrate 1000000
   ```

## 📚 详细文档

查看完整文档：
```bash
cat debug/README.md
```
