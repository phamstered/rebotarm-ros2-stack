# ROS 2 Humble CI base image

Published as **`ghcr.io/phamstered/rebotarm-ros2-ci`**.

This image is a pre-warmed Humble environment for GitHub Actions. It is **not** a
runtime or hardware bring-up image.

## What is baked

- Base: official `ros:humble-ros-base` (Ubuntu Jammy + ROS 2 Humble)
- `rosdep install` from every `package.xml` under `src/` (including
  `src/plugin/microros_stm32h723_bridge`)
- Extra CI tools: `python3-pytest`, `python3-yaml`, `ccache`
- `rosdep` skip-keys (external / Jetson overlays, not in this repo):
  `stararm102_description`, `micro_ros_agent`

## What is **not** baked

- Full workspace source or a `colcon` install/build tree
- Hardware bring-up (CAN, Jetson, STM32 micro-ROS agent)
- Gazebo GUI / live MoveIt demos
- Desk (`rebotarm-debug-desk`) or Executor (`rebotarm-executor`) concerns

The Dockerfile copies **only** `package.xml` files into a temporary workspace for
`rosdep`, then deletes that tree so the image stays lean.

## Local build

From the repository root:

```bash
docker build -f docker/ci/Dockerfile -t ghcr.io/phamstered/rebotarm-ros2-ci:humble .
```

Optional date tag:

```bash
docker tag ghcr.io/phamstered/rebotarm-ros2-ci:humble \
  ghcr.io/phamstered/rebotarm-ros2-ci:humble-$(date +%Y%m%d)
```

## How CI publishes it

`.github/workflows/ci-image.yml` builds with Buildx and pushes to GHCR on:

- `workflow_dispatch`
- `push` to `main` when `docker/ci/**` or any `**/package.xml` changes

Tags pushed:

- `humble` (floating)
- `humble-<short-sha>` (immutable-ish)
- optionally `humble-YYYYMMDD`

The workflow prints the image **digest**. Prefer consuming
`ghcr.io/phamstered/rebotarm-ros2-ci@sha256:<digest>` rather than a floating tag.

## Two-step rollout

1. **This change** — Dockerfile + publish workflow only. `.github/workflows/ci.yml`
   still runs on `ubuntu-22.04` with `ros-tooling/setup-ros`.
2. **Follow-up PR** — switch `ci.yml` to the container and **digest-pin**
   (`container: ghcr.io/phamstered/rebotarm-ros2-ci@sha256:…`). Do not switch until
   the image exists and the digest is recorded.
