# rebotarm-ros2-stack

ROS 2 Humble **software stack for reBotArm B601** (6-DoF + gripper): robot description and Gazebo Fortress simulation, MoveIt2 motion planning, a ros2_control `SystemInterface` to STM32H723 over micro-ROS (MIT impedance, Pinocchio gravity feed-forward, Joint4/payload calibration), a legacy CAN MIT follower driver, and a configurable joint-state relay / DDS domain bridge.

**Leader–follower teleoperation is one application path** (via `joint_state_relay`), not the whole product surface. Prefer reading `src/` over older teleop-only narratives.

| | |
|---|---|
| **Product / repo** | rebotarm-ros2-stack (formerly `leader_follower`) |
| **Repository** | [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) |
| **ROS distro** | Humble |
| **Simulation** | Ignition Gazebo 6 (Fortress) + gz_ros2_control |
| **User docs** | [https://phamstered.github.io/rebotarm-ros2-stack/](https://phamstered.github.io/rebotarm-ros2-stack/) |

## 相关仓库 / 职责

| 仓库 | 职责 | 文档 |
|---|---|---|
| [`phamstered/robot-debug-desk`](https://github.com/phamstered/robot-debug-desk) | 上位机 | [Pages](https://phamstered.github.io/robot-debug-desk/) |
| [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) | ROS 2 controller（本仓） | [Pages](https://phamstered.github.io/rebotarm-ros2-stack/) |
| [`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor) | FreeRTOS + micro-ROS 固件 | [Pages](https://phamstered.github.io/rebotarm-executor/) |

本仓库是 ROS 2 控制栈，不是固件，也不是上位机 UI。

## Stack layers (code-aligned)

| Layer | What lives here |
|---|---|
| **Description / sim** | `rebotarm_gazebo` — URDF, meshes, Ignition worlds, gazebo controllers |
| **Motion planning** | `rebotarm_moveit_config` — MoveIt2 demo (mock) and Gazebo demo |
| **ros2_control hardware** | `microros_stm32h723_bridge` — `MicroRosSystem` + MIT modes HOLD/TRACK/RETURN/DISABLE |
| **ros2_control controllers** | teleop / trajectory / gripper position; `GravityCompensationController`; `Joint4CalibrationController` |
| **CAN driver (non-plugin)** | `rebotarm_hw_driver` — `/joint_commands` → MIT loop on `can0` |
| **Application / bridging** | `joint_state_relay` — joint map, domain bridge, gripper endpoint calibration |

Backends (pick one per physical arm): **Gazebo** · **CAN** · **micro-ROS**. Details: **[控制栈与后端](https://phamstered.github.io/rebotarm-ros2-stack/controllers)** · [docs/controllers.md](docs/controllers.md).

## Packages

| Package | Role |
|---|---|
| `rebotarm_gazebo` | URDF/Xacro, meshes, Gazebo, controllers, RViz helpers |
| `rebotarm_moveit_config` | MoveIt2 planning config and launches |
| `plugin/microros_stm32h723_bridge` | ros2_control `SystemInterface`, gravity + Joint4 calibration controllers, msgs, launches |
| `rebotarm_hw_driver` | Real follower CAN/MIT driver node |
| `joint_state_relay` | Leader→follower mapping (`gazebo` / `real` / `ros2_control`), domain bridge |

Architecture write-up: **[架构与包](https://phamstered.github.io/rebotarm-ros2-stack/architecture)** · [docs/architecture.md](docs/architecture.md).

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

| Scenario | Where | Command |
|---|---|---|
| Gazebo follower | Ubuntu | `./scripts/start_gazebo.sh` (`--headless=True` if needed) |
| MoveIt mock demo | any | `ros2 launch rebotarm_moveit_config demo.launch.py` |
| MoveIt + Gazebo | Ubuntu | `ros2 launch rebotarm_moveit_config gazebo_demo.launch.py` |
| Leader + relay → sim | Jetson | `./scripts/start_jetson.sh --mode=gazebo` |
| Leader + CAN follower | Jetson | `./scripts/start_jetson.sh --mode=real` |
| micro-ROS ros2_control (teleop/trajectory/gravity/calibration) | Jetson | `./scripts/start_arm_teleop.sh` (see `--position-controller`, `--gravity`, `--mode=calibration`) |
| Sync + remote build | Dev machine | `./scripts/sync_and_build.sh` (uses `rsync --delete` — review first) |

Full steps: **[构建与运行](https://phamstered.github.io/rebotarm-ros2-stack/build-run)** · [docs/build-run.md](docs/build-run.md).

### Preview docs locally

```bash
npm install
npm run docs:dev
```

Static build: `npm run docs:build`.

## CI

`.github/workflows/ci.yml` on `push` / `pull_request` → `main` (Ubuntu 22.04 · ROS 2 Humble):

- `colcon build` (including `microros_stm32h723_bridge`)
- pytest: `joint_state_relay/test`
- ament pytest: micro-ROS bridge config / source contracts

**Not covered:** live hardware, Gazebo GUI, Jetson bring-up. Docs deploy via `.github/workflows/deploy-docs.yml`. See **[CI](https://phamstered.github.io/rebotarm-ros2-stack/ci)** · [docs/ci.md](docs/ci.md).

## Docs and guidelines

- User site: [https://phamstered.github.io/rebotarm-ros2-stack/](https://phamstered.github.io/rebotarm-ros2-stack/)
- Architecture: [docs/architecture.md](docs/architecture.md)
- Build & run: [docs/build-run.md](docs/build-run.md)
- Control stack: [docs/controllers.md](docs/controllers.md)
- CI: [docs/ci.md](docs/ci.md)
- Contributor / agent notes: [AGENTS.md](AGENTS.md)
- Historical notes (not authoritative vs code): [CLAUDE.md](CLAUDE.md)
