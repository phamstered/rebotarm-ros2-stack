# leader_follower — 主从臂跟随系统

## 项目概述

移动真实主臂 (Star-Arm-102)，从臂 (reBotArm B601) 实时跟随。

**两种运行场景：**
1. 真实主臂 → Gazebo 仿真从臂 (Ubuntu 192.168.55.100)
2. 真实主臂 → 真实从臂 (Jetson 192.168.2.124)

## 设备拓扑

| 设备 | SSH | 角色 |
|------|-----|------|
| Jetson ORIN Nano 8G | `ssh damiao` @ 192.168.2.124 | 主臂驱动 + relay |
| Ubuntu 22.04 AMD64 | `ssh ubuntu` @ 192.168.55.100 | Gazebo 仿真 |
| macOS | 开发机 | 代码同步 |

## ROS2 包结构

- `joint_state_relay` — 关节状态中继（real/gazebo 双模式，domain bridge）
- `rebotarm_gazebo` — 从臂 URDF + Gazebo 仿真 + ros2_control
- `rebotarm_hw_driver` — 从臂硬件驱动（MIT 阻抗控制）
- `rebotarm_moveit_config` — MoveIt2 运动规划配置

## 启动脚本

| 脚本 | 运行位置 | 用途 |
|------|---------|------|
| `scripts/start_jetson.sh` | Jetson | 主臂驱动 + relay_node |
| `scripts/start_gazebo.sh` | Ubuntu | Gazebo 仿真 + 控制器 + bridge + relay |
| `scripts/start_follower_rviz.sh` | Ubuntu | 从臂 RViz2 可视化 |
| `scripts/sync_and_build.sh` | Mac | 同步代码到 Jetson+Ubuntu 并编译 |
| `scripts/debug_gazebo_data_flow.sh` | Ubuntu | 诊断 Gazebo 数据流 |
| `scripts/prepare_urdf.py` | Ubuntu | 生成带 world 固定关节的 URDF |
| `scripts/measure_zero_offset.py` | Jetson | 测量夹爪零点偏移 |
| `scripts/measure_gripper.py` | Jetson | 测量夹爪完整映射 |
| `scripts/monitor_gripper.py` | Jetson | 监控夹爪电机状态 |

**用法示例：**
```bash
# Jetson: gazebo 模式 (仅主臂+relay，从臂在 Ubuntu 仿真)
./scripts/start_jetson.sh --mode=gazebo

# Jetson: real 模式 (主臂+从臂硬件+relay)
./scripts/start_jetson.sh --mode=real

# Ubuntu: 有 GUI
./scripts/start_gazebo.sh

# Ubuntu: 无头模式 (SSH)
./scripts/start_gazebo.sh --headless=True

# 诊断数据流
./scripts/debug_gazebo_data_flow.sh

# 测量夹爪零点偏移 (需要先启动从臂驱动)
python3 scripts/measure_zero_offset.py --domain 0 --duration 5

# 测量夹爪完整映射 (交互式)
python3 scripts/measure_gripper.py --domain 0

# 监控夹爪电机状态
python3 scripts/monitor_gripper.py --duration 60 --domain 0
```

## 技术栈

- **ROS2 Humble** + **Ignition Gazebo 6** (Fortress)
- **gz_ros2_control 0.7.19** — 硬件插件: `ign_ros2_control/IgnitionSystem`
- **ros2_control** — `JointTrajectoryController` + `JointStateBroadcaster`
- **MoveIt2** — OMPL 规划器，KDL 运动学
- **DART** 物理引擎

## 关键约定

### Gazebo 仿真

- 必须用 `ign gazebo`（非 `gz sim`），因 gz_ros2_control 0.7.19 编译于 ign-gazebo6
- 模型 spawn 用 `ign service`（非 `ros_gz_sim create`，后者与 ign6 不兼容）
- `LD_LIBRARY_PATH` 必须包含 `/opt/ros/humble/lib`
- `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 必须指向 `/opt/ros/humble/lib`
- URDF 中 `package://` 需替换为绝对路径
- base_link 通过 world fixed joint 固定在世界原点

### 关节映射

- 1:7 映射：leader joint1-6 → follower joint1-6，leader joint7_left → follower gripper_finger_left_joint
- 符号翻转：`follower_pos = sign * leader_pos + offset`
- 夹爪缩放：`gripper_pos = 0.0127 * leader_gripper_pos` (rad → m)
- Real 模式：`motor_pos = prismatic_pos * 100.0` (m → rad)
- sign 配置在 `relay.yaml` 中，经实际校准确认

### 夹爪映射校准

**配置文件**:
- Gazebo: `src/joint_state_relay/config/relay.yaml` (sign=0.0127, offset=0.0)
- Real: `src/joint_state_relay/config/relay_real.yaml` (sign=0.0212, offset=0.003619)

**变换公式**:
```
follower_pos(m) = sign × leader_pos(rad) + offset
```

**单位**:
- Leader: 弧度 (rad) - 旋转关节
- Follower: 米 (m) - 直线关节 (prismatic)
- sign: m/rad

**真实从臂驱动转换**:
```python
motor_pos = prismatic_pos(m) × -133.0 → 电机位置(rad)
```

**完整转换链**:
```
主臂角度(rad) → sign → 夹爪位移(m) → ×(-133) → 电机角度(rad)
```

**校准工具**:
- `scripts/measure_zero_offset.py` - 测量零点偏移，计算修正的 offset
- `scripts/measure_gripper.py` - 交互式测量完整映射 (sign 和 offset)

**校准步骤**:
1. 启动从臂驱动: `./scripts/start_jetson.sh --mode=real`
2. 测量零点偏移: `python3 scripts/measure_zero_offset.py --domain 0 --duration 5`
3. 根据输出更新配置文件
4. 重新编译: `colcon build --packages-select joint_state_relay`
5. 重启 relay 节点

### DDS 隔离

- `ROS_DOMAIN_ID=0`: Jetson 主臂
- `ROS_DOMAIN_ID=1`: Ubuntu 从臂仿真
- `domain_bridge` 节点桥接 `/joint_states` (domain 0 → `/leader/joint_states` domain 1)

### relay_node 模式

- `mode: "real"` — 发布 `JointState` 到 `/joint_commands`（真实从臂）
- `mode: "gazebo"` — 通过 `FollowJointTrajectory` action 发送到 Gazebo 控制器

## 规则

- 所有代码及依赖（URDF/Mesh 等）必须拷贝到 leader_follower 项目中
- SSH 到 Jetson: `ssh damiao`，SSH 到 Ubuntu: `ssh ubuntu`
- `start_jetson.sh` 必须在 Jetson 运行，`start_gazebo.sh` 必须在 Ubuntu 运行
- 新脚本必须 `chmod +x`
