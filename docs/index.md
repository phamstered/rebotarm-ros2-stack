---
layout: home

hero:
  name: rebotarm-ros2-stack
  text: reBotArm B601 的 ROS 2 控制栈
  tagline: 描述 / 仿真 · ros2_control · micro-ROS · MoveIt · CAN 驱动 · 关节中继
  actions:
    - theme: brand
      text: 架构与包
      link: /architecture
    - theme: alt
      text: 构建与运行
      link: /build-run
    - theme: alt
      text: 控制栈
      link: /controllers
    - theme: alt
      text: GitHub
      link: https://github.com/phamstered/rebotarm-ros2-stack

features:
  - title: ros2_control 真机后端
    details: MicroRosSystem 将 position/effort 接口合成为 MIT JointCommand；可选 Pinocchio 重力前馈与 Joint4/负载标定。
  - title: 仿真与运动规划
    details: Gazebo Fortress + gz_ros2_control，以及 MoveIt2（mock demo / Gazebo demo）完整规划链路。
  - title: 多后端与应用层
    details: CAN MIT 驱动、关节状态中继（gazebo / real / ros2_control）、DDS domain bridge；主从遥操只是其中一种用法。
---

## 这是什么

**rebotarm-ros2-stack**（[`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack)）是面向 **reBotArm B601**（6 轴 + 夹爪）的 ROS 2 Humble **机械臂软件栈**：从 URDF/仿真、ros2_control 硬件接口、轨迹与遥操控制器、重力补偿与标定，到 MoveIt 规划与主从关节中继。

仓库曾用名 `leader_follower`；主从跟随（teleop）仍是受支持的应用路径，但 **不是** 本仓库的全部能力边界。文档以 `src/` 中的包、launch、控制器与硬件插件为准。

| | |
|---|---|
| **产品 / 仓库名** | rebotarm-ros2-stack |
| **目标手臂** | reBotArm B601（joint1–6 + `gripper_finger_left_joint`） |
| **ROS 发行版** | Humble |
| **仿真** | Ignition Gazebo 6 (Fortress) + gz_ros2_control |
| **真机路径** | micro-ROS（STM32H723）或 CAN MIT（`rebotarm_hw_driver`） |

固件（STM32 / FreeRTOS / micro-ROS 节点）见兄弟仓库 **rebotarm-executor**；上位机 UI 见 **robot-debug-desk**。本站只覆盖本仓库的 ROS 2 控制栈。

## 能力一览（对齐代码）

| 能力 | 主要代码位置 |
|------|----------------|
| 机器人描述、meshes、Gazebo 世界 | `rebotarm_gazebo` |
| MoveIt2 规划 / demo / Gazebo 联调 | `rebotarm_moveit_config` |
| ros2_control `SystemInterface`（MIT · HOLD/TRACK/RETURN/DISABLE） | `microros_stm32h723_bridge::MicroRosSystem` |
| 重力前馈（effort-only，Pinocchio） | `GravityCompensationController` |
| Joint4 + camera_link 负载多姿态标定 | `Joint4CalibrationController` |
| 位置遥操 / 轨迹控制器 + 夹爪 | `arm_teleop_controller`、`arm_trajectory_controller`、`gripper_teleop_controller` |
| CAN 侧 MIT 驱动（非 ros2_control 插件） | `rebotarm_hw_driver` |
| 关节映射中继、domain bridge、夹爪端点校准 | `joint_state_relay` |

## 快速入口

1. [架构与包](./architecture) — 分层与 `src/` 职责
2. [构建与运行](./build-run) — colcon、仿真、MoveIt、Jetson / micro-ROS 启动
3. [控制栈与后端](./controllers) — Gazebo / CAN / micro-ROS 与控制器矩阵
4. [CI 说明](./ci) — GitHub Actions 覆盖范围
5. [ros2_control 关联机制](./ros2_control_association) — controller ↔ hardware 接口匹配

## 本机预览文档

```bash
npm install
npm run docs:dev
```

构建静态站点：

```bash
npm run docs:build
```
