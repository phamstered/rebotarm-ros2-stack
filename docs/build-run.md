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

## Teleop discovery and signal shutdown

`start_arm_teleop.sh` selects the installed `joint_state_relay/config/fastdds_udp.xml` (source fallback before install) when no explicit `FASTRTPS_DEFAULT_PROFILES_FILE` or `FASTDDS_DEFAULT_PROFILES_FILE` exists. It uses UDPv4 for the processes it starts; it does not remove live `/dev/shm` files or change micro-ROS Agent systemd configuration. This avoids stale/deleted SHM lock files isolating an already-running desktop Bridge from newly started nodes. An explicitly supplied XML profile keeps precedence; UDP may have different camera bandwidth/CPU costs.

Rebuild `joint_state_relay` when updating this startup script:

```bash
colcon build --symlink-install --packages-select joint_state_relay
source install/setup.bash
```

The managed leader process now runs `ros2 run joint_state_relay leader_driver --ros-args -r __ns:=/leader -p lock:=disable`. This adapter uses the existing external `robo_driver.robo_driver.Arm_contorl` node and its cleanup method; the Star Arm SDK, serial path, joint mapping and local vendor changes stay external. Both leader and relay treat `KeyboardInterrupt` / `ExternalShutdownException` as expected ROS signal shutdown, clean the node/context, and propagate other failures. A native `RCLError` from a wait-set shutdown race is ignored only after `rclpy.ok()` confirms that the context has closed; the same error in a live context remains a failure. The prior safe-stop confirmation before process cleanup, STM32 fault/feedback checks and startup acknowledgements still apply. STM32 JointState timeout remains an error.

Read-only / isolated checks (use a nonzero ROS domain, no serial hardware):

```bash
colcon test --packages-select joint_state_relay
colcon test-result --verbose
ROS_DOMAIN_ID=222 python3 src/joint_state_relay/test/smoke_lifecycle_signals.py
```

The signal smoke test substitutes a synthetic vendor module and exercises repeated SIGINT/SIGTERM for the actual installed leader/relay executables, plus genuine initialization failure propagation.

## 保存点位的真机 MoveIt 服务

已通过原启动流程激活真实轨迹控制器后，单独运行：

```bash
ros2 launch rebotarm_moveit_config real_move_group.launch.py
```

该入口不激活硬件或切换控制器，只使用 OMPL；复用 micro-ROS URDF、当前相机安装变换、SRDF 和米制夹爪反馈 `/joint_states`。上位机点位执行使用 `/move_action`，再由 MoveIt 调用唯一的 `arm_trajectory_controller/follow_joint_trajectory`，不直接执行模拟 demo。

环境碰撞依据 Planning Scene 中的已知障碍物；上位机显示 RGBD 点云不等于点云已注册为碰撞环境。可在无硬件的隔离域运行真实规划/障碍物拒绝验证：

```bash
ROS_DOMAIN_ID=226 python3 src/rebotarm_moveit_config/test/test_real_moveit_planning.py
```

测试启动自己的 move_group，只调用状态有效性与规划服务，不发送执行目标；退出时清理临时进程。

## 真机轨迹速度前馈与启动连续性

真机 arm profile 的 joint1–joint6 导出 position、effort、velocity 命令接口；`arm_trajectory_controller` 必须认领 position+velocity，保留独立重力 effort 控制器。JTC 插值速度仅在 TRACK 且该接口已认领时发送到 STM32。Teleop/VLA/夹爪的位置控制保持原行为；velocity 接口停止/重新启动时清零残值，HOLD/RETURN/DISABLE 的目标速度恒为 0。旧 position+effort profile 仍兼容，不会从位置差分猜测速度。

更新插件、xacro 和 arm_controllers.yaml 后，需要按既有安全停机流程重启控制栈并重新激活 JTC，仅编译不改变运行中的接口。不要在机械臂维持输出时直接替换/重启进程。与 Desk 的启动连续性门禁配对使用：参考与实测位置大静差会拒绝新点位，不伪造实际起点、不放宽碰撞/起点校验。当前增益与重力模型参数未自动改变；是否改善低速黏滑须用相同点位、负载和监控采集复核。

软件链路隔离验证（仅模拟 MCU，禁止生产 Domain）：

```bash
ROS_DOMAIN_ID=229 python3 src/plugin/microros_stm32h723_bridge/test/smoke_trajectory_velocity.py
```

该测试启动自己的 mock STM32 与真实 JTC/硬件插件，确认速度非零、切回 Teleop 清零、重新激活无残值及 DISABLE 清零；退出仅清理自建进程。隔离测试通过不代表真机振动已经消除。
