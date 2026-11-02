# ros2_control Controller 与 Hardware 关联机制

## 核心原理

`ros2_control` 中，controller 和 hardware component **没有显式绑定**。它们通过 **关节名 + 接口类型名** 自动匹配：

```
Hardware 导出接口                    Controller 声明需求
  joint1/position (cmd)  ◄────────  joint1/position (command_interface)
  joint1/position (state) ────────► joint1/position (state_interface)
  joint1/effort   (cmd)  ◄────────  joint1/effort   (gravity / calibration)
```

`controller_manager` 在 activate 时完成匹配。本仓库中，同一套关节上可以同时存在 **position** 与 **effort** 命令接口；位置控制器与重力控制器因此可以并存，而两个都声明 effort 的控制器不能同时激活。

## 本仓库实际 Hardware 插件

| 场景 | `<plugin>`（代码中） | 定义位置 |
|------|----------------------|----------|
| MoveIt mock demo | `mock_components/GenericSystem` | `rebotarm_moveit_config/config/rebotarm.ros2_control.xacro` |
| Gazebo 仿真 | `ign_ros2_control/IgnitionSystem` | `rebotarm_gazebo/urdf/rebotarm.ros2_control.xacro` |
| STM32 micro-ROS 真机 | `microros_stm32h723_bridge/MicroRosSystem` | `microros_stm32h723_bridge/urdf/rebotarm.arm.micro_ros.xacro` |

**注意：** 仓库内 **不存在** `rebotarm_hw_interface/RebotArmSystem`。CAN 真机路径走 `rebotarm_hw_driver` Python 节点，订阅 `/joint_commands`，不经过上述 SystemInterface。

`MicroRosSystem` 为 joint1–6（可选夹爪）同时导出：

- command：`position`、`velocity`、`effort`
- state：`position`、`velocity`、`effort`

write 时把位置、已认领的速度（仅 TRACK）、模式增益（HOLD/TRACK/RETURN/DISABLE/IDLE 增益组）与限幅后的 effort 合成固定长度 `JointCommand`。没有轨迹积分项。速度前馈在接口停止/重启时清零；HOLD、RETURN、DISABLE 的目标速度为 0。

## 配置层级

### 1. URDF/xacro — 硬件提供的接口

示例（micro-ROS arm，节选概念）：

```xml
<ros2_control name="Stm32H723ArmSystem" type="system">
  <hardware>
    <plugin>microros_stm32h723_bridge/MicroRosSystem</plugin>
    <param name="command_topic">/stm32/joint_commands</param>
    <param name="state_topic">/stm32/joint_states</param>
    <param name="gains_file">...</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- joint2–6 · 可选 gripper_finger_left_joint -->
</ros2_control>
```

### 2. 控制器 YAML — Controller 需要的接口

- Gazebo / MoveIt demo：`ros2_controllers.yaml` → `arm_controller`（position）。Gazebo 另有 `gripper_controller`
- micro-ROS arm：`arm_controllers.yaml` → `arm_teleop_controller` / `vla_position_controller`（position）、`arm_trajectory_controller`（position+velocity）、可选 `gripper_teleop_controller` 或 `gripper_trajectory_controller`，以及 `gravity_compensation_controller`（effort）

`joints` 与接口名必须与 xacro 一致。

### 3. MoveIt — 只关心 action 名

`moveit_controllers.yaml` 将规划结果发到 `arm_trajectory_controller` 的 `FollowJointTrajectory`。真机 `real_move_group.launch.py` 使用这个名字；`enable_gripper:=true` 时再加入 `gripper_trajectory_controller`。Gazebo 与 MoveIt demo 的控制器仍叫 `arm_controller`，不会自动改成上述名字。

## 数据流（micro-ROS）

```
命令源（teleop 话题 / trajectory action / MoveIt）
  ▼
position controller  → 写 joint*/position
trajectory controller → 另写 joint*/velocity（仅其认领的接口，且仅 TRACK）
gravity controller   → 写 joint*/effort   （可选）
  ▼
MicroRosSystem::write()
  → JointCommand{mode, position, velocity, kp, kd, effort, sequence}
  ▼
micro-ros-agent → STM32
  ▼
JointState / HardwareStatus → MicroRosSystem::read()
  ▼
joint_state_broadcaster → /joint_states
```

## 互斥与安全

| 规则 | 原因 |
|------|------|
| `gravity_compensation_controller` 与 `joint4_calibration_controller` 不同时 active | 争用同一 effort 接口 |
| `arm_teleop_controller`、`arm_trajectory_controller`、`vla_position_controller` 只能有一个 active | 争用 joint1–6 position |
| `gripper_teleop_controller` 与 `gripper_trajectory_controller` 只能有一个 active | 争用夹爪 position |
| 串口仅一个 micro-ROS agent | 设备独占 |
| deactivate 前调用 `safe_stop` | 避免固件命令丢失锁存故障 |

## 排查

| 现象 | 检查 |
|------|------|
| controller 无法 claim 接口 | `ros2 control list_hardware_interfaces` |
| 插件未加载 | `microros_stm32h723_bridge.xml` 中的 class name 是否与 xacro `<plugin>` 一致 |
| controller 停在 inactive | `ros2 control list_controllers`；用 spawner 或 `set_controller_state` |
| 重力无效 | 是否已 activate gravity 控制器；`gravity_scales` 是否为 0 |
