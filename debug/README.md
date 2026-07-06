# Joint4 高频振动诊断工具包

## 目录结构

```
debug/
├── README.md                    # 本文档
├── run_diagnostics.sh          # 总入口脚本
│
├── 01_capture_vibration.sh     # 步骤1: 录制振动数据
├── 02_analyze_bag.py           # 步骤2: 分析 bag 数据
├── 03_check_communication.sh   # 步骤3: 检查通信质量
├── 04_isolate_source.sh        # 步骤4: 隔离问题源
├── 05_compare_topics.py        # 步骤5: 对比 topic 数据
├── 06_monitor_joint4.py        # 步骤6: 实时监控 joint4
├── 07_check_motor_params.sh    # 步骤7: 检查电机参数
└── 08_analyze_frequency.py     # 步骤8: 频谱分析
```

## 使用流程

### 快速诊断（推荐）

```bash
# 在 Jetson 上运行完整诊断
./debug/run_diagnostics.sh
```

### 分步诊断

#### 步骤 1: 录制振动数据
```bash
# 在 Jetson 上运行，操作主臂到振动位置
./debug/01_capture_vibration.sh

# 输出: vibration_<timestamp>/
#   ├── joint_states.bag
#   ├── joint_commands.bag
#   └── metadata.json
```

#### 步骤 2: 分析录制数据
```bash
# 分析 bag 文件中的抖动
python3 debug/02_analyze_bag.py vibration_<timestamp>/

# 输出: 分析报告，包含抖动检测、统计信息
```

#### 步骤 3: 检查通信质量
```bash
# 实时检查 DDS 通信
./debug/03_check_communication.sh

# 输出: 频率、延迟、丢包率
```

#### 步骤 4: 隔离问题源
```bash
# 引导式测试，判断问题在数据层还是控制层
./debug/04_isolate_source.sh
```

#### 步骤 5: 对比 topic 数据
```bash
# 对比主臂原始数据和 relay 输出
python3 debug/05_compare_topics.py --duration 10

# 输出: 差异分析，判断 relay 是否引入抖动
```

#### 步骤 6: 实时监控 joint4
```bash
# 实时显示 joint4 状态，带抖动检测
python3 debug/06_monitor_joint4.py --plot

# 输出: 实时曲线 + 抖动报警
```

#### 步骤 7: 检查电机参数
```bash
# 检查 joint4 电机配置
./debug/07_check_motor_params.sh

# 输出: 当前 Kp/Kd、力矩限制等
```

#### 步骤 8: 频谱分析
```bash
# 对录制数据进行 FFT 分析
python3 debug/08_analyze_frequency.py vibration_<timestamp>/

# 输出: 频谱图、主频识别
```

## 常见问题

### Q: 振动频率 ~50Hz
A: 可能是控制环频率问题，检查电机驱动的控制周期

### Q: 振动频率 ~100-200Hz
A: 可能是 Kd 增益过高，尝试降低 30%

### Q: 只在特定角度振动
A: 可能是机械间隙或重力补偿不足

### Q: 随机振动
A: 可能是通信丢包或编码器噪声

## 注意事项

1. **安全第一** - 调试时确保从臂工作空间内无人
2. **备份参数** - 修改前记录原始参数
3. **逐步验证** - 每次只改一个参数
4. **录制数据** - 修改前后都录制对比数据
