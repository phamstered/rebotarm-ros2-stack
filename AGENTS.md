# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble workspace for the **reBotArm B601 control stack** (description, simulation, ros2_control / micro-ROS, MoveIt, CAN driver, joint relay). Packages live under `src/`:

- `rebotarm_gazebo/`: URDF/Xacro, meshes, Gazebo Fortress worlds, controllers, RViz helpers.
- `rebotarm_moveit_config/`: MoveIt2 planning config, mock demo and Gazebo demo launches.
- `plugin/microros_stm32h723_bridge/`: ros2_control `MicroRosSystem` hardware plugin, gravity and Joint4 calibration controllers, msgs, launches, mock/smoke scripts.
- `rebotarm_hw_driver/`: CAN MIT follower driver node (`/joint_commands` → motors).
- `joint_state_relay/`: leader→follower joint mapping (`gazebo` / `real` / `ros2_control` modes), DDS domain bridge, gripper calibration helper.

Operational helpers are in `scripts/`; vibration and communication diagnostics are in `debug/`. Requirements and design notes belong in `specs/`. Keep package-owned resources beside their package so `colcon` installs them correctly. Prefer `src/` over historical teleop-only docs when they disagree.

## Build, Test, and Development Commands

Run ROS commands from a shell with Humble sourced:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

Use `colcon build --packages-select joint_state_relay` (or `microros_stm32h723_bridge`) for a quicker package-only build. Simulation: `./scripts/start_gazebo.sh` on the Ubuntu host. CAN bring-up: `./scripts/start_jetson.sh --mode=real`. micro-ROS ros2_control pipeline: `./scripts/start_arm_teleop.sh`. `./scripts/sync_and_build.sh` synchronizes `src/` and `scripts/` to remote machines and builds there; review it before running because `rsync --delete` removes remote-only files.

## Coding Style & Naming Conventions

Use four-space indentation and PEP 8 conventions for Python. Name modules, functions, ROS nodes, topics, and YAML keys with `snake_case`; use `PascalCase` for classes. Launch files follow `<purpose>.launch.py`, and package names remain lowercase with underscores. Keep ROS parameters in package `config/*.yaml` files rather than embedding environment-specific values in nodes. No formatter or linter is configured, so keep imports organized and changes consistent with neighboring code.

## Testing Guidelines

Package-local automated tests live under `test/`:

- `joint_state_relay/test/` — pytest (relay YAML contracts + gripper mapping helpers)
- `plugin/microros_stm32h723_bridge/test/` — ament pytest (gains / msg / launch source contracts)

Name Python tests `test_<behavior>.py`. GitHub Actions (`.github/workflows/ci.yml`) runs `colcon build` plus these suites on Ubuntu 22.04 / ROS 2 Humble; it does **not** exercise hardware, Gazebo GUI, or Jetson bring-up. For new behavior, extend those tests, then locally: build the affected package, run `colcon test` (or the matching pytest path), and smoke-test the relevant launch mode. Hardware changes should document the device, ROS domain, and observed result.

## Commit & Pull Request Guidelines

Recent history uses Conventional Commit prefixes such as `feat:` and `fix:`, often followed by concise Chinese descriptions. Keep each commit focused and use the same `<type>: <summary>` form. Pull requests should describe the scenario affected (real hardware, Gazebo, MoveIt, or docs), list verification commands, call out configuration or calibration changes, and link related issues. Include screenshots for RViz/Gazebo visual changes and logs for runtime or hardware fixes.
