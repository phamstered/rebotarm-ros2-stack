# rebotarm-ros2-stack

ROS 2 Humble **主从臂跟随**控制栈：移动真实主臂（Star-Arm-102），从臂（reBotArm B601）实时跟随。支持 Gazebo 仿真从臂与 Jetson 真机从臂（CAN / micro-ROS）。

| | |
|---|---|
| **产品 / 仓库名** | rebotarm-ros2-stack（曾用名 `leader_follower`） |
| **仓库** | [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) |
| **ROS 发行版** | Humble |
| **仿真** | Ignition Gazebo 6 (Fortress) + gz_ros2_control |
| **用户文档** | [https://phamstered.github.io/rebotarm-ros2-stack/](https://phamstered.github.io/rebotarm-ros2-stack/)（GitHub Pages） |

> 本文档以 **rebotarm-ros2-stack** 作为仓库与产品名；本地工作区目录曾为 `leader_follower`，以当前仓库名为准。

## 兄弟仓库（角色边界）

| 仓库 | 角色 | 文档 |
|---|---|---|
| **本仓库** [`rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) | ROS 2 控制器栈（relay · ros2_control / micro-ROS · Gazebo · MoveIt） | [Pages](https://phamstered.github.io/rebotarm-ros2-stack/) |
| [`robot-debug-desk`](https://github.com/phamstered/robot-debug-desk) | 上位机 UI（macOS · SSH 隧道 + WebSocket Bridge） | [Pages](https://phamstered.github.io/robot-debug-desk/) |
| [`rebotarm-executor`](https://github.com/phamstered/rebotarm-executor) | FreeRTOS / micro-ROS 固件（STM32） | [Pages](https://phamstered.github.io/rebotarm-executor/) |

本仓库 **不是** 固件工程，也 **不是** 桌面主机 UI。

## 架构一览

| 层 | 组件 |
|---|---|
| **主臂** | Star-Arm-102 · Jetson（串口驱动 + `joint_states`） |
| **中继** | `joint_state_relay`（gazebo / real / ros2_control 模式 · domain bridge） |
| **从臂 · 仿真** | Gazebo Fortress + `ros2_control` · Ubuntu |
| **从臂 · CAN** | `rebotarm_hw_driver` · MIT 阻抗 · `can0` |
| **从臂 · micro-ROS** | `microros_stm32h723_bridge` · agent 串口 · STM32H723 |

**两种运行场景**

1. 真实主臂 → Gazebo 仿真从臂（Jetson domain 0 + Ubuntu domain 1）
2. 真实主臂 → 真实从臂（Jetson：CAN 或 micro-ROS）

交互式说明与包职责见用户文档 **[架构与包](https://phamstered.github.io/rebotarm-ros2-stack/architecture)**。源文件：[docs/architecture.md](docs/architecture.md)。

## 包概览

| 包 | 职责 |
|---|---|
| `joint_state_relay` | 主→从关节映射、DDS domain bridge、YAML / launch |
| `rebotarm_hw_driver` | 真实从臂 CAN/MIT 驱动 |
| `rebotarm_gazebo` | URDF/Xacro、meshes、Gazebo、控制器与 RViz |
| `rebotarm_moveit_config` | MoveIt2 规划与控制器配置 |
| `plugin/microros_stm32h723_bridge` | ros2_control `SystemInterface`（micro-ROS · 重力补偿 / Joint4 标定） |

控制路径（Gazebo / CAN / micro-ROS）对照见 **[控制路径](https://phamstered.github.io/rebotarm-ros2-stack/controllers)** · [docs/controllers.md](docs/controllers.md)。

## 构建与运行

在已 source ROS 2 Humble 的 shell 中：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

| 场景 | 位置 | 命令 |
|---|---|---|
| 主臂 + relay（仿真从臂） | Jetson | `./scripts/start_jetson.sh --mode=gazebo` |
| 主臂 + 从臂硬件 + relay | Jetson | `./scripts/start_jetson.sh --mode=real` |
| Gazebo 仿真从臂 | Ubuntu | `./scripts/start_gazebo.sh`（无头：`--headless=True`） |
| 同步并远端编译 | 开发机 | `./scripts/sync_and_build.sh`（含 `rsync --delete`，先审阅） |

完整步骤、Gazebo 约定与校准脚本见 **[构建与运行](https://phamstered.github.io/rebotarm-ros2-stack/build-run)** · [docs/build-run.md](docs/build-run.md)。

### 本机预览文档

```bash
npm install
npm run docs:dev
```

静态构建：`npm run docs:build`。

## CI

`.github/workflows/ci.yml` 在 `push` / `pull_request` → `main` 上运行（Ubuntu 22.04 · ROS 2 Humble）：

- `colcon build`（含 `microros_stm32h723_bridge`）
- pytest：`joint_state_relay/test`
- ament pytest：micro-ROS 桥配置 / 源码契约

**不覆盖** 真机上电、Gazebo GUI、Jetson bring-up。文档站点由 `.github/workflows/deploy-docs.yml` 部署到 GitHub Pages。详见 **[CI 说明](https://phamstered.github.io/rebotarm-ros2-stack/ci)** · [docs/ci.md](docs/ci.md)。

## 文档与规范

- 用户文档站点：[https://phamstered.github.io/rebotarm-ros2-stack/](https://phamstered.github.io/rebotarm-ros2-stack/)
- 架构与包：[docs/architecture.md](docs/architecture.md)
- 构建与运行：[docs/build-run.md](docs/build-run.md)
- 控制路径：[docs/controllers.md](docs/controllers.md)
- CI：[docs/ci.md](docs/ci.md)
- 贡献 / Agent 规范：[AGENTS.md](AGENTS.md)
- 项目笔记（历史）：[CLAUDE.md](CLAUDE.md)
