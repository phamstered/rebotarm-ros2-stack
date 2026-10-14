# 构建与运行

在已 source ROS 2 Humble 的 shell 中操作。路径与主机角色见 [架构与包](./architecture)。

## 本地编译与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

单包迭代：

```bash
colcon build --packages-select joint_state_relay
colcon build --packages-select microros_stm32h723_bridge
```

`rosdep` 安装依赖时，跳过仓库外 overlay：

```bash
rosdep install --from-paths src --ignore-src -y -r \
  --rosdistro humble \
  --skip-keys "stararm102_description micro_ros_agent"
```

CI 同等覆盖见 [CI 说明](./ci)。

## 同步到远端并编译

```bash
./scripts/sync_and_build.sh
```

会 rsync `src/` 与 `scripts/` 到 Jetson 与 Ubuntu 并远程编译。脚本含 `rsync --delete`，运行前请确认不会误删远端文件。

## 场景：仿真与规划（无需真机从臂）

| 场景 | 命令 | 说明 |
|------|------|------|
| Gazebo 从臂 | `./scripts/start_gazebo.sh`（无头：`--headless=True`） | Ubuntu；`ROS_DOMAIN_ID=1` |
| 从臂 RViz | `./scripts/start_follower_rviz.sh` | 可视化 |
| MoveIt mock demo | `ros2 launch rebotarm_moveit_config demo.launch.py` | 无 Gazebo / 无硬件 |
| MoveIt + Gazebo | `ros2 launch rebotarm_moveit_config gazebo_demo.launch.py` | 规划到仿真控制器 |

### Gazebo 约定

- 使用 `ign gazebo`（不要用 `gz sim`）：当前 gz_ros2_control 针对 ign-gazebo6
- 模型 spawn 用 `ign service`（不要用与 ign6 不兼容的 `ros_gz_sim create`）
- `LD_LIBRARY_PATH` 与 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 需包含 `/opt/ros/humble/lib`
- URDF 中 `package://` 需替换为绝对路径（`scripts/prepare_urdf.py`）
- `base_link` 通过 world fixed joint 固定在世界原点

## 场景：主从跟随（应用层）

| 位置 | 命令 |
|------|------|
| Jetson（仿真从臂） | `./scripts/start_jetson.sh --mode=gazebo` |
| Jetson（CAN 真机从臂） | `./scripts/start_jetson.sh --mode=real` |
| Ubuntu（配合 gazebo 模式） | `./scripts/start_gazebo.sh` |

## 场景：micro-ROS ros2_control 真机

前置：systemd `micro-ros-agent-stm32h723`（或等价 agent）已占用串口；固件 active mask 满足关节集合（臂 `0x3F`，含夹爪 `0x7F`）。

```bash
# 带守卫的完整管线（默认 teleop 位置控制器）
./scripts/start_arm_teleop.sh --clean=true

# 轨迹控制器 + 重力前馈
./scripts/start_arm_teleop.sh \
  --position-controller=trajectory --gravity=true --clean=true

# 纯重力调参（无位置控制器；硬件先 inactive）
./scripts/start_arm_teleop.sh \
  --position-controller=none --gravity=true --clean=true

# Joint4 多姿态标定
./scripts/start_arm_teleop.sh --mode=calibration --clean=true

# 仅反馈、不启命令路径
ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py state_only:=true
```

开发用 mock（无 agent）：

```bash
ros2 run microros_stm32h723_bridge mock_stm32_node.py
ros2 launch microros_stm32h723_bridge stm32h723_bridge.launch.py start_agent:=false
```

单电机冒烟（固件 mask=`0x01`）：

```bash
ros2 run microros_stm32h723_bridge single_motor_smoke_test.py \
  --target 0.05 --kp 2.0 --kd 0.2 --duration 2.0
```

## 校准与诊断

| 工具 | 用途 |
|------|------|
| `scripts/measure_zero_offset.py` / `measure_gripper*.py` | 关节 / 夹爪映射 |
| `ros2 run joint_state_relay calibrate_gripper` | 主臂↔STM32 夹爪端点（只读采样） |
| `scripts/monitor_topics.sh` / `monitor_gripper.py` | 话题与夹爪监控 |
| `scripts/debug_gazebo_data_flow.sh` | Gazebo 数据流 |
| `scripts/sample_moveit_tracking_error.py` | MoveIt 跟踪误差采样 |
| `debug/` | Joint4 振动、通信、电机参数等诊断流水线 |

校准后更新对应 `joint_state_relay/config/*.yaml` 或硬件 xacro 中的 scale/offset，再 `colcon build` 并重启节点。

## SSH 约定

- Jetson：`ssh damiao`
- Ubuntu：`ssh ubuntu`
- 新脚本需 `chmod +x`
