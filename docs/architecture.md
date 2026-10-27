# 架构与包

本文档从仓库 `src/` 的实现归纳栈结构。历史「仅主从遥操」叙述若与代码冲突，以代码为准。

## 分层视图

```
应用 / 命令源
  · MoveIt2 (move_group → FollowJointTrajectory)
  · joint_state_relay (leader → follower；mode: gazebo | real | ros2_control)
  · 直接话题/action（arm_teleop / trajectory / 标定服务）
        │
        ▼
ros2_control 控制器层（controller_manager）
  · JointTrajectoryController / JointGroupPositionController
  · JointStateBroadcaster
  · GravityCompensationController（effort-only）
  · Joint4CalibrationController（与重力控制器互斥）
        │  接口名匹配：jointN/position · jointN/effort
        ▼
硬件后端（三选一，勿混用同一物理臂）
  · Gazebo: ign_ros2_control / mock_components
  · micro-ROS: microros_stm32h723_bridge/MicroRosSystem
  · CAN 直驱: rebotarm_hw_driver（绕过 ros2_control 插件，吃 /joint_commands）
        │
        ▼
物理 / 仿真
  · Ignition Gazebo 6 · STM32H723（agent 串口）· can0 MIT 电机
```

**主从跟随**只是「应用 / 命令源」里的一种：`joint_state_relay` 把主臂 `joint_states` 映射后写入从臂后端。同一后端也可由 MoveIt、轨迹 action、重力标定脚本驱动。

## 设备拓扑（部署参考）

| 设备 | 角色（代码中的用法） |
|------|----------------------|
| Jetson ORIN Nano | 主臂串口驱动；CAN 或 micro-ROS 从臂；relay / `start_arm_teleop.sh` |
| Ubuntu 22.04 AMD64 | Gazebo Fortress 仿真从臂（`ROS_DOMAIN_ID=1`） |
| 开发机 | `scripts/sync_and_build.sh` 同步与远端编译 |

连接：主臂串口；从臂 `can0`（CAN 路径）或 `/dev/ttyACM0` + `micro-ros-agent`（micro-ROS 路径）。仿真场景用 `domain_bridge` 跨 DDS domain。

## `src/` 包（实现真相）

| 包 | 类型 | 职责（对齐代码） |
|----|------|------------------|
| **`rebotarm_gazebo`** | ament_cmake | B601 URDF/Xacro、meshes、Gazebo 世界、`ros2_controllers.yaml`、`rebotarm_gazebo.launch.py`、RViz / gz↔ROS 桥脚本 |
| **`rebotarm_moveit_config`** | ament_cmake | MoveIt2：SRDF、OMPL、KDL、`demo.launch.py`（mock）、`gazebo_demo.launch.py`、控制器与限位 YAML |
| **`microros_stm32h723_bridge`** | ament_cmake（`src/plugin/`） | `hardware_interface::SystemInterface` 插件；自定义 msg；Pinocchio 重力库；两个 `controller_interface` 插件；arm / bridge launch；mock / smoke / republisher 脚本 |
| **`rebotarm_hw_driver`** | ament_python | 经 `reBotArm_control_py` 的 CAN MIT 从臂节点：订阅 `/joint_commands`，发布 `joint_states`，内建 RETURN/HOLD/TRACK 与重力前馈 |
| **`joint_state_relay`** | ament_python | 主→从映射与限速；三种 mode；`domain_bridge`；`calibrate_gripper` |

辅助目录（非独立 ROS 包）：`scripts/`（启动与校准）、`debug/`（振动/通信诊断）、`systemd/`（micro-ROS agent 单元）、`specs/`。

## micro-ROS 桥：硬件与控制器

插件注册（pluginlib）：

| 类 | 基类 | 作用 |
|----|------|------|
| `microros_stm32h723_bridge/MicroRosSystem` | `SystemInterface` | 7 槽位 MIT：读 `/stm32/joint_states`，写 `/stm32/joint_commands`；模式 HOLD/TRACK/RETURN/DISABLE；`safe_stop` 服务 |
| `…/GravityCompensationController` | `ControllerInterface` | 只占 **effort** 命令接口；Pinocchio `GravityCompensation` |
| `…/Joint4CalibrationController` | `ControllerInterface` | 多姿态回归（gravity_scale / 偏置 / camera_link 等效质量）；与重力控制器 **互斥** |

`stm32h723_arm.launch.py` 用 launch 参数组合位置控制器与重力/标定；其中 `robot_state_publisher` 订阅 `/stm32/joint_states` 生成动态 TF，使用与 Robot View 相同的实时反馈。`/joint_states` 仍由 ros2_control 广播，但硬件 unconfigured 时可能继续输出最后一次位置，不能用于相机点云的实时 TF：

| 参数 | 取值 | 效果 |
|------|------|------|
| `position_controller` | `teleop` / `trajectory` / `none` | 激活 teleop、轨迹，或纯 effort 调参 |
| `enable_gravity_controller` | bool | 与位置控制器可并存（不同接口） |
| `enable_joint4_calibration` | bool | 加载标定控制器（通常 inactive，由守卫脚本激活） |
| `enable_gripper` | bool | 导出夹爪 position/effort；`required_joint_mask` 0x7F vs 0x3F |
| `state_only` | bool | 仅 `stm32_state_republisher`，不启 ros2_control 命令路径 |

消息契约：`JointCommand.msg` / `HardwareStatus.msg`（固定 `float32[7]`，含 sequence、mode、mask、fault）。

## 关节中继三种 mode

| `mode` | 配置文件 | 输出 |
|--------|----------|------|
| `gazebo` | `relay.yaml` | `FollowJointTrajectory` → Gazebo `arm_controller` / `gripper_controller` |
| `real` | `relay_real.yaml` | `sensor_msgs/JointState` → `/joint_commands`（CAN 驱动） |
| `ros2_control` | `relay_microros_arm.yaml` | `Float64MultiArray` → `/arm_teleop_controller/commands`（及夹爪 teleop） |

映射：`follower = sign * leader + offset`；leader `joint7_left` → `gripper_finger_left_joint`。仿真跨机时 `domain_bridge`：domain 0 `/joint_states` → domain 1 `/leader/joint_states`。

## MoveIt 与仿真

- **独立规划验证**：`rebotarm_moveit_config/demo.launch.py` → `mock_components/GenericSystem`
- **仿真规划**：`gazebo_demo.launch.py` 包含 `rebotarm_gazebo.launch.py` + `move_group`
- Gazebo 控制器：`arm_controller` + `gripper_controller`（`JointTrajectoryController`，update_rate 500 Hz）

## 技术栈

- ROS 2 Humble · ros2_control · controller_manager
- Ignition Gazebo 6 (Fortress) · gz_ros2_control
- MoveIt2（OMPL · KDL · MoveItConfigsBuilder）
- Pinocchio（重力与标定）
- micro-ROS Agent（串口 XRCE-DDS；本仓库不包含固件）
- 可选外部：`stararm102_description` / `robo_driver`（主臂，Jetson overlay）
