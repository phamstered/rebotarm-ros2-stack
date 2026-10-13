---
layout: home

hero:
  name: rebotarm-ros2-stack
  text: ROS 2 主从臂跟随控制栈
  tagline: joint_state_relay · ros2_control / micro-ROS · MoveIt · Gazebo Fortress
  actions:
    - theme: brand
      text: 架构与包
      link: /architecture
    - theme: alt
      text: 构建与运行
      link: /build-run
    - theme: alt
      text: 控制路径
      link: /controllers
    - theme: alt
      text: GitHub
      link: https://github.com/phamstered/rebotarm-ros2-stack

features:
  - title: 双场景跟随
    details: 真实主臂 → Gazebo 仿真从臂，或真实主臂 → 真实从臂（Jetson CAN / micro-ROS）。
  - title: ros2_control 插件
    details: microros_stm32h723_bridge 作为 SystemInterface，MIT 阻抗 + 重力补偿，串口由 micro-ros-agent 负责。
  - title: MoveIt + Gazebo
    details: rebotarm_moveit_config + Ignition Gazebo 6（Fortress）+ JointTrajectoryController。
---

## 这是什么

**rebotarm-ros2-stack**（仓库 [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack)，曾用名 `leader_follower`）是 ROS 2 Humble 工作空间：主臂（Star-Arm-102）运动时，从臂（reBotArm B601）实时跟随。

| | |
|---|---|
| **产品 / 仓库名** | rebotarm-ros2-stack |
| **ROS 发行版** | Humble |
| **仿真** | Ignition Gazebo 6 (Fortress) + gz_ros2_control |
| **真机主机** | Jetson ORIN Nano（主臂驱动 + relay / 从臂） |
| **仿真主机** | Ubuntu 22.04 AMD64（Gazebo） |

固件与 STM32 侧细节见兄弟仓库 **rebotarm-executor**；本站只覆盖 ROS 2 控制器栈文档。

## 快速入口

1. [架构与包](./architecture) — `src/` 包职责与设备拓扑
2. [构建与运行](./build-run) — colcon、仿真与 Jetson 启动脚本
3. [控制路径](./controllers) — CAN 真机驱动 vs micro-ROS ros2_control
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
