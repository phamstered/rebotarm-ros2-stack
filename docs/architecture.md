# 架构与包

## 设备拓扑

| 设备 | SSH / 地址 | 角色 |
|------|------------|------|
| Jetson ORIN Nano 8G | `ssh damiao` @ 192.168.2.124 | 主臂驱动 + relay；real 模式下亦跑从臂 |
| Ubuntu 22.04 AMD64 | `ssh ubuntu` @ 192.168.55.100 | Gazebo 仿真从臂 |
| macOS | 开发机 | 代码同步（`scripts/sync_and_build.sh`） |

**连接摘要**

- Jetson ↔ 主臂：串口（如 `/dev/ttyCH341USB0`）
- Jetson ↔ 从臂（CAN 路径）：`can0`
- Jetson ↔ 从臂（micro-ROS 路径）：串口 + `micro-ros-agent`（如 `/dev/ttyACM0`）
- 三机同一局域网

## 两种运行场景

```
场景 A — 仿真从臂
  主臂 (Jetson, domain 0)
       │ joint_states
       ▼
  domain_bridge → /leader/joint_states (domain 1)
       │
       ▼
  relay_node (mode=gazebo) → FollowJointTrajectory / arm_controller
       │
       ▼
  Gazebo Fortress + ros2_control (Ubuntu)

场景 B — 真实从臂
  主臂 (Jetson)
       │
       ▼
  relay_node (mode=real | ros2_control)
       │
       ├── CAN 路径: /joint_commands → rebotarm_hw_driver
       └── micro-ROS 路径: arm_teleop / trajectory → microros_stm32h723_bridge
```

## `src/` 包

| 包 | 职责 |
|----|------|
| `joint_state_relay` | 主→从关节映射、DDS domain bridge、YAML 配置与 launch（gazebo / real / microros） |
| `rebotarm_hw_driver` | 真实从臂 CAN/MIT 驱动与硬件 launch |
| `rebotarm_gazebo` | URDF/Xacro、meshes、Gazebo 模型、控制器与 RViz |
| `rebotarm_moveit_config` | MoveIt2 规划与控制器配置 |
| `plugin/microros_stm32h723_bridge` | ros2_control `SystemInterface`：micro-ROS JointState ↔ MIT JointCommand；重力补偿 / Joint4 标定控制器 |

包内资源放在各自包目录，以便 `colcon` 正确安装。需求与设计笔记在 `specs/`；振动与通信诊断在 `debug/`。

## DDS 隔离（仿真场景）

| Domain | 用途 |
|--------|------|
| `ROS_DOMAIN_ID=0` | Jetson 主臂 |
| `ROS_DOMAIN_ID=1` | Ubuntu 从臂仿真 |

`domain_bridge` 将 domain 0 的 `/joint_states` 桥到 domain 1 的 `/leader/joint_states`。

## 关节映射（概要）

- 1:7 映射：leader `joint1`–`joint6` → follower `joint1`–`joint6`；leader `joint7_left` → `gripper_finger_left_joint`
- 变换：`follower_pos = sign * leader_pos + offset`（配置见 `joint_state_relay/config/*.yaml`）
- 夹爪：leader 弧度 → follower 直线位移（m）；real 驱动侧再换算到电机角度

具体 sign/offset 以各 YAML 为准，校准工具见 `scripts/measure_*.py`。

## 技术栈

- **ROS 2 Humble**
- **Ignition Gazebo 6** (Fortress) + **gz_ros2_control**（插件 `ign_ros2_control/IgnitionSystem`）
- **ros2_control** — `JointTrajectoryController` + `JointStateBroadcaster`（及 teleop / gravity 控制器）
- **MoveIt2** — OMPL + KDL
- **Pinocchio**（micro-ROS 桥重力前馈）
