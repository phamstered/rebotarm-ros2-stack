# 构建与运行

在已 source ROS 2 Humble 的 shell 中操作。

## 本地编译与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

单包加快迭代：

```bash
colcon build --packages-select joint_state_relay
# 或
colcon build --packages-select microros_stm32h723_bridge
```

`rosdep` 安装依赖时，仓库外包（Jetson overlay）可跳过：

```bash
rosdep install --from-paths src --ignore-src -y -r \
  --rosdistro humble \
  --skip-keys "stararm102_description micro_ros_agent"
```

## 同步到远端并编译

开发机：

```bash
./scripts/sync_and_build.sh
```

会 rsync `src/` 与 `scripts/` 到 Jetson 与 Ubuntu 并远程编译。使用前请确认：脚本含 `rsync --delete`，会删除远端多余文件。

## Jetson：主臂 + relay

必须在 Jetson 上执行：

```bash
# 仿真从臂：仅主臂 + relay（从臂在 Ubuntu Gazebo）
./scripts/start_jetson.sh --mode=gazebo

# 真实从臂：主臂 + 从臂硬件驱动 + relay
./scripts/start_jetson.sh --mode=real
```

## Ubuntu：Gazebo 仿真从臂

必须在 Ubuntu 仿真机上执行：

```bash
./scripts/start_gazebo.sh
# SSH / 无头
./scripts/start_gazebo.sh --headless=True
```

可视化：

```bash
./scripts/start_follower_rviz.sh
```

### Gazebo 约定（重要）

- 使用 `ign gazebo`（不要用 `gz sim`）：当前 gz_ros2_control 针对 ign-gazebo6
- 模型 spawn 用 `ign service`（不要用与 ign6 不兼容的 `ros_gz_sim create`）
- `LD_LIBRARY_PATH` 与 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 需包含 `/opt/ros/humble/lib`
- URDF 中 `package://` 需替换为绝对路径（见 `scripts/prepare_urdf.py`）
- `base_link` 通过 world fixed joint 固定在世界原点

## 常用诊断脚本

| 脚本 | 位置 | 用途 |
|------|------|------|
| `debug_gazebo_data_flow.sh` | Ubuntu | 诊断 Gazebo 数据流 |
| `monitor_topics.sh` | 任意 | 话题监控 |
| `measure_zero_offset.py` / `measure_gripper.py` | Jetson | 夹爪零点与映射校准 |
| `monitor_gripper.py` | Jetson | 夹爪电机状态 |
| `start_arm_teleop.sh` | Jetson | micro-ROS / ros2_control 遥操与标定入口 |
| `debug/` | Jetson | Joint4 振动等诊断流水线 |

校准示例：

```bash
./scripts/start_jetson.sh --mode=real
python3 scripts/measure_zero_offset.py --domain 0 --duration 5
# 更新 relay_real.yaml 后
colcon build --packages-select joint_state_relay
# 重启 relay
```

## SSH 约定

- Jetson：`ssh damiao`
- Ubuntu：`ssh ubuntu`
- 新脚本需 `chmod +x`
