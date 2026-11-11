# Jetson Docker runtime (Humble)

Host workspaces and `scripts/start_*.sh` remain available. Build the runtime on
Jetson Ubuntu 22.04/aarch64 with Docker Engine (or on Linux amd64):

```bash
./scripts/docker-runtime.sh build
ROS_DOMAIN_ID=221 ./scripts/docker-runtime.sh run bash /opt/rebotarm/scripts/verify-runtime.sh
```

The image builds this checkout into `/opt/rebotarm`; it does not reuse host
`build/` or `install/`. Host parity uses the same source revision:

```bash
source /opt/ros/humble/setup.bash
colcon build --executor sequential
ROS_DOMAIN_ID=221 ./scripts/verify-runtime.sh
```

Run the two checks sequentially. Reserve the verification domain (220–232) for
tests; they use localhost discovery and mock MCU, never production domain 0.
The verifier runs colcon tests plus arm and gripper mock trajectory smokes.

`run` requires an explicit command and uses host networking. For example, launch
only the planning server after the required hardware/controllers are ready:

```bash
./scripts/docker-runtime.sh run ros2 launch rebotarm_moveit_config real_move_group.launch.py
```

No command runs automatically when building the image. Do not run duplicate
hardware/controller managers on host and Docker for the same arm/domain.
Containers have their own PID namespace; the runner does not use `--pid=host`.
No systemd is started inside the container.

For camera/serial/CAN access, opt into root plus device privileges:

```bash
STACK_DOCKER_PRIVILEGED=1 ./scripts/docker-runtime.sh run \
  ros2 launch rebotarm_gazebo left_wrist_camera.launch.py
```

The image runs as root; privilege opt-in also exposes `/dev` and read-only
`/run/udev`. Configure the CAN interface on the Jetson host. This CPU image does
not configure CUDA/NVIDIA runtime, Gazebo GUI or desktop forwarding.

External `stararm102_description` and `micro_ros_agent` are not bundled. Supply
Humble/aarch64 overlays built for this environment using `ROBOT_EXTERNAL_ROOT`
and `ROBOT_EXTERNAL_SETUP`. The root is mounted read-only at its original
absolute path so symlink-install resources work; it must include all symlink
targets. Do not mount host install artifacts over `/opt/rebotarm`.

`start_arm_teleop.sh` checks a local systemd service or local Agent serial
process before checking fresh STM32 feedback. An Agent running only on the host
is not visible in this container PID namespace. For that script, provision the
Agent overlay and run its serial process in the same container. Its existing
feedback, controller, gravity and calibration gates still apply. The runtime
runner does not automatically start or restart an Agent.

`STACK_DOCKER_IMAGE` overrides the image tag. Bridge builds on this image using
the paired debug-desk `docker/jetson/Dockerfile`; keep both source revisions in
the deployment record. Desktop SSH operations still target the host unless
explicit container execution support is configured in the desktop version.
