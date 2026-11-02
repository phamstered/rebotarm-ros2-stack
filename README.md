# reBotArm B601 ROS 2 控制栈

[`rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) — ROS 2 Humble **software stack for reBotArm B601** (6-DoF + gripper): robot description and Gazebo Fortress simulation, MoveIt2 motion planning, a ros2_control `SystemInterface` to STM32H723 over micro-ROS (MIT impedance, Pinocchio gravity feed-forward, Joint4/payload calibration), a legacy CAN MIT follower driver, and a configurable joint-state relay / DDS domain bridge.

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
| [`phamstered/rebotarm-debug-desk`](https://github.com/phamstered/rebotarm-debug-desk) | 上位机 | [Pages](https://phamstered.github.io/rebotarm-debug-desk/) |
| [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) | ROS 2 controller（本仓） | [Pages](https://phamstered.github.io/rebotarm-ros2-stack/) |
| [`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor) | FreeRTOS + micro-ROS 固件 | [Pages](https://phamstered.github.io/rebotarm-executor/) |

本仓库是 ROS 2 控制栈，不是固件，也不是上位机 UI。

## Stack layers (code-aligned)

| Layer | What lives here |
|---|---|
| **Description / sim** | `rebotarm_gazebo` — URDF, meshes, Ignition worlds, gazebo controllers |
| **Motion planning** | `rebotarm_moveit_config` — MoveIt2 mock demo, Gazebo demo, and real `real_move_group.launch.py` |
| **ros2_control hardware** | `microros_stm32h723_bridge` — `MicroRosSystem` + MIT modes HOLD/TRACK/RETURN/DISABLE |
| **ros2_control controllers** | teleop / trajectory / gripper position and gripper trajectory; `GravityCompensationController`; `Joint4CalibrationController`; `VlaPositionController` |
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
| 左臂腕部 RealSense RGBD | Jetson | `ros2 launch rebotarm_gazebo left_wrist_camera.launch.py`（多相机时指定 `serial_no:=...`） |
| Sync + remote build | Dev machine | `./scripts/sync_and_build.sh` (uses `rsync --delete` — review first) |

左腕相机入口固定使用 `camera_namespace=cameras/left_arm`、`camera_name=wrist`，启用彩色、深度、对齐深度和帧同步。Topic 前缀为 `/cameras/left_arm/wrist/`；`camera_link → left_arm/wrist_link` 的零位姿静态 TF 将 RealSense 光学帧连接到现有 URDF hand-eye 外参。真机启动文件让 `robot_state_publisher` 从经过时间戳检查和单调 20 Hz 限流的 `/robot_feedback/tf_joint_states` 生成动态 TF，与 Robot View 的机械臂反馈保持一致，避免 ros2_control 硬件未配置时 `/joint_states` 仍输出旧姿态。启动前需要安装 `realsense2_camera`、`tf2_ros` 并完成 `rebotarm_gazebo` 构建。

真实轨迹控制器已显式激活后，可单独启动点位规划服务：

```bash
ros2 launch rebotarm_moveit_config real_move_group.launch.py
```

该入口只启动 `move_group`，复用真实 URDF、SRDF 与 `/joint_states`（夹爪单位须已换算成米），不创建硬件/ControllerManager，不自动激活控制器。默认执行目标是 `arm_trajectory_controller`；夹爪轨迹已激活时加 `enable_gripper:=true`，同时映射 `gripper_trajectory_controller`。规划组为 `arm`、`gripper`、`arm_with_gripper`（`link6` 位姿加夹爪关节，无单独七轴 IK）。真实六关节速度、加速度分别限于 2 rad/s、2 rad/s²；配套上位机请求使用 1.0 缩放。夹爪规划上限为 0.02 m/s、0.04 m/s²。此处是规划上限，不会修改固件限幅、MIT 增益或重力补偿。起点容差为 0.01 rad。环境障碍物需通过 Planning Scene 输入，碰撞检查只覆盖模型和已知场景。`demo.launch.py` 仍只用于隔离仿真。Gazebo 控制器名仍是 `arm_controller`，与这份真机 MoveIt 映射不同。

Full steps: **[构建与运行](https://phamstered.github.io/rebotarm-ros2-stack/build-run)** · [docs/build-run.md](docs/build-run.md).

相机安装姿态以 2026-09-12 MoveIt 手眼标定导出中实际使用的四元数为准：`link6 → camera_link`，`xyzw=[0.598366, 0.00397963, 0.801167, 0.00856162]`。URDF 将归一化四元数转换为 RPY，pitch 为负值；原导出中注释掉的 RPY 与该四元数不等价。RealSense 光学 `+Z` 经相机 body `+X` 指向腕部前方（`link6 +Z` 侧）。该修复恢复已有标定结果；重新安装相机后仍需重新标定。

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
- `colcon test`: micro-ROS bridge, `rebotarm_gazebo`, `rebotarm_moveit_config`
- isolated smokes: mock-MCU trajectory velocity, calibrated gripper trajectory, and MoveIt full-pose planning (no execution)

`.github/workflows/ci-image.yml` publishes `ghcr.io/phamstered/rebotarm-ros2-ci`, but `ci.yml` still installs Humble with `ros-tooling/setup-ros` on `ubuntu-22.04`.

**Not covered:** live hardware, Gazebo GUI, Jetson bring-up. Docs deploy via `.github/workflows/deploy-docs.yml`. See **[CI](https://phamstered.github.io/rebotarm-ros2-stack/ci)** · [docs/ci.md](docs/ci.md).

## Docs and guidelines

- User site: [https://phamstered.github.io/rebotarm-ros2-stack/](https://phamstered.github.io/rebotarm-ros2-stack/)
- Architecture: [docs/architecture.md](docs/architecture.md)
- Build & run: [docs/build-run.md](docs/build-run.md)
- Control stack: [docs/controllers.md](docs/controllers.md)
- CI: [docs/ci.md](docs/ci.md)
- Contributor / agent notes: [AGENTS.md](AGENTS.md)
- Historical notes (not authoritative vs code): [CLAUDE.md](CLAUDE.md)

### Teleop DDS and shutdown behavior

The Teleop entry uses a process-scoped Fast DDS UDP profile unless an explicit DDS XML profile is supplied. Rebuild `joint_state_relay` after updating the script: its `leader_driver` adapter runs the installed external Star Arm node with normal ROS shutdown handling, preserving serial/joint configuration and real errors. See [build/run](docs/build-run.md#teleop-discovery-and-signal-shutdown). Safe-stop and firmware acknowledgement checks remain required; no live SHM files are deleted.

### STM32 feedback clock admission

`stm32_state_republisher.py` preserves the original acquisition stamp. Zero/malformed stamps,
more than 50 ms in the future, 1 s or more old, and regressions greater than 50 ms
are rejected; duplicate/minor out-of-order samples are dropped. ROS clock jumps relative
to monotonic time greater than 250 ms restart admission. Recovery requires 200 ms of
continuous valid increasing feedback. Silence for 1 s marks the stream stale.
Firmware diagnostic v1 (16 words) remains compatible. V2 (24 words) additionally requires
synchronized firmware state, last-sync age <15 s and diagnostics received within 3 s;
a firmware epoch change restarts recovery. Each relay publishes read-only JSON health at
`~/time_sync_status`, including state, generation, source offset and
receipt age; fresh communication alone does not prove synchronized source time.

Normalized feedback retains its original rate and stamp. Only RobotStatePublisher uses
the separate volatile `/robot_feedback/tf_joint_states` output, limited to 20 Hz using
monotonic time. RSP's `ignore_timestamp=true` bypasses its old-epoch throttle *after* the
relay guard, retaining the source stamp on TF; corrected timestamps can therefore publish
without restarting RSP/controllers. TF consumers must clear already-cached future data on
epoch changes (the desktop Bridge handles its own buffer); changing the guard cannot erase
historical buffers in external consumers. Upgrade requires installing the helper script,
republisher and launch together and restarting these read-only nodes through the normal
safe bring-up flow. Controllers and firmware watchdogs are unchanged.

### 六轴规划上限 2/2 的核验

通用 `config/joint_limits.yaml` 与真实 `real_move_group.launch.py` 均显式启用
六轴 2 rad/s、2 rad/s²，默认速度/加速度缩放均为 1.0。夹爪仍为
0.02 m/s、0.04 m/s²，MIT Kp/Kd、固件及控制器反馈安全门不随规划上限改变。
安装新配置后必须经现有受管理规划器流程重启 MoveIt，旧进程的 RobotModel 不会
因修改 YAML 自动刷新。不能仅凭参数值声称内存中的 RobotModel 已更新。

配置包在存在 `moveit_ros_planning` 时构建只读测试工具 `test_robot_model_limits`，
从真实启动参数加载 RobotModel，核对六轴以及夹爪实际变量边界；不创建硬件或
Action 目标。隔离真实规划测试可设置 `ROBOT_MODEL_LIMITS_PROBE` 为该二进制路径，
在规划前先检查实际模型边界。部署后先核对进程身份、启动参数和模型边界，再做
只读状态验证；不自动下发机械臂动作。

隔离验证示例（不执行轨迹）：

```bash
source install/setup.bash
ROBOT_MODEL_LIMITS_PROBE="$PWD/build/rebotarm_moveit_config/test_robot_model_limits" \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=226 \
  python3 src/rebotarm_moveit_config/test/test_real_moveit_planning.py
```

实际进程参数也可用 `ros2 param dump /move_group` 只读导出，再以同一参数文件运行
模型探针；探针关闭参数服务及事件发布，没有控制器、Action 客户端或运动目标。
运行前须确认导出文件根节点名称与探针节点 `move_group` 一致。
