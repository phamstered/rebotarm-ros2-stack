# microros_stm32h723_bridge

ROS 2 Humble `hardware_interface::SystemInterface` for a seven-joint STM32H723 micro-ROS motor controller. The plugin receives `JointState` feedback and publishes fixed-size MIT commands. Serial transport remains owned by `micro-ros-agent`.

Gravity feed-forward is provided by the separately switchable
`gravity_compensation_controller`. Its per-joint `gravity_scales` and
`effort_limits` are configured in `config/single_joint_controllers.yaml`.
For supported-arm tuning without the leader or relay, run:

```bash
./scripts/start_joint1_teleop_relay.sh --mode=gravity --clean=true
```

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select microros_stm32h723_bridge
source install/setup.bash
```

The target must provide ROS 2 Humble, ros2_control, Pinocchio, yaml-cpp, and micro-ROS Agent. The model path and gain file are supplied through the hardware parameters in `urdf/rebotarm.micro_ros.xacro`.

## Mock test

Start the mock STM32 endpoint, then launch ros2_control without an agent:

```bash
ros2 run microros_stm32h723_bridge mock_stm32_node.py
ros2 launch microros_stm32h723_bridge stm32h723_bridge.launch.py start_agent:=false
```

The launch file defaults to `start_agent:=false` because real deployments use
the systemd-managed Agent. Never start both processes against the same serial
device.

Before deactivating the hardware, request a safe stop:

```bash
ros2 service call /microros_stm32h723_hardware/safe_stop std_srvs/srv/Trigger '{}'
ros2 topic echo /stm32/status
```

## Single-motor bring-up

For the first hardware test, configure firmware with `active_joint_mask=0x01` and connect only `joint1`. Do not launch the seven-joint hardware plugin; run the restricted topic-level test instead:

```bash
micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
ros2 run microros_stm32h723_bridge single_motor_smoke_test.py \
  --target 0.05 --kp 2.0 --kd 0.2 --duration 2.0
```

The tool refuses targets outside ±0.10 rad, gains above its conservative smoke-test limits, and any firmware status whose active mask is not exactly `0x01`. Keep the motor unloaded and retain a physical emergency stop.

## Joint1-3 ros2_control bring-up

After the topic-level smoke test passes, start the dedicated bring-up profile while
the systemd Agent is running:

```bash
ros2 launch microros_stm32h723_bridge stm32h723_single_joint.launch.py
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

This profile requires firmware whose active mask contains `0x3F` and exports the
position interfaces for joint1-6. RETURN/HOLD gains match the legacy follower
driver. The current TRACK experiment uses 40.0/3.0 for joint1-3, 40.0/1.5 for
joint4, and 10.0/1.5 for joint5-6. IDLE is 0.0/1.0 for all six joints. Gravity
feed-forward is enabled in normal control mode and limited to 5 N*m for
joint1-3 and 3 N*m for joint4-6. State-only mode never loads this control path.

## Joint1-6 teleoperation

The bring-up launch activates `joint1_teleop_controller` by default and
loads `joint1_trajectory_controller` in the inactive state. Start the relay to
begin streaming leader joint1-6 commands:

The trajectory controller accepts partial goals, so a trajectory naming only
`joint1` holds the other five joints at their current positions. Its action is
`/joint1_trajectory_controller/follow_joint_trajectory`.

```bash
./scripts/start_joint1_teleop_relay.sh \
  --start-controller=true --start-leader=true \
  --ros-domain-id=0 --leader-timeout=30
```

The helper checks `/leader/joint_states`, `/joint_states`, and the loaded
Controller before starting the relay. Keep the relay in one terminal and
verify the published command from another terminal.

It also requires fresh `/stm32/joint_states` and `/stm32/status`, recovers an
unconfigured HardwareInterface, verifies that all six position interfaces are
available and claimed, and waits for a real message on the plural command topic
`/joint1_teleop_controller/commands`. Any failed stage stops startup instead of
leaving a controller-only process tree behind.

Startup is reported as nine self-check stages, beginning with the leader serial
driver and `/leader/joint_states`, then checking the systemd micro-ROS Agent,
STM32 mask/fault status, controllers, HardwareInterface, follower feedback,
relay output, `/stm32/joint_commands`, and the STM32 command-sequence
acknowledgement. The script prints an end-to-end path summary only after every
stage passes.

Leader initialization may take longer than 30 seconds, so its default timeout
is 90 seconds. Child launches run in dedicated process groups for reliable
cleanup. Before stopping an active ros2_control process, the helper requests
the hardware safe-stop service and waits for STM32 `MODE_DISABLE`; this avoids
triggering the firmware's latched command-loss fault during normal shutdown.
The bring-up host feedback timeout is 500 ms to tolerate occasional
micro-ROS serial/DDS scheduling gaps; the STM32 firmware retains its faster CAN
feedback and command-loss safety checks.

By default, the helper stops stale joint1 relay processes, restarts the leader
and controller processes that it owns, and refreshes the ROS 2 daemon before
launching. Pass `--clean=false` only when intentionally reusing an existing
runtime.

Use `--start-leader=false` (the default) when a separately managed leader
driver already publishes `/leader/joint_states`. With `true`, the helper starts
`robo_driver` in the `/leader` namespace with `lock:=disable` and stops only
that child process when the relay exits.

The helper also starts the joint1-6 ros2_control launch by default. If a
`ros2_control_node` process already exists, it is reused. Pass
`--start-controller=false` when Controller Manager is managed separately; the
script then fails instead of starting a second instance.

The relay consumes `/leader/joint_states` and streams a six-element
`Float64MultiArray` ordered as joint1 through joint6 to
`/joint1_teleop_controller/commands`. Absolute mapping is enabled by default,
so follower encoder zeros and mapping offsets must be calibrated before startup.
Relay-side position rate limiting is 1.0 rad/s and starts from measured follower
state, while joint limits, deadbands, and leader timeout handling remain active.
The joint state broadcaster
explicitly publishes joint1 through joint6 in that order. Move the leader slowly
during initial testing and keep a physical emergency stop available.

## Read-only calibration view

Stop `rebotarm_moveit_config demo.launch.py` before starting the real follower
view; its mock Controller Manager otherwise publishes zero-valued joint states.
Start the STM32 Agent, then launch the bridge in state-only mode:

```bash
ros2 launch microros_stm32h723_bridge \
  stm32h723_single_joint.launch.py state_only:=true
```

This mode starts `robot_state_publisher` and republishes joint1-6 from
`/stm32/joint_states` to `/joint_states` in deterministic order. It does not
start Controller Manager, load MicroRosSystem, activate controllers, or publish
`/stm32/joint_commands`. Use it to compare the real follower pose with the URDF
before writing motor zero offsets.

Return to trajectory control with:

```bash
ros2 control switch_controllers \
  --deactivate joint1_teleop_controller \
  --activate joint1_trajectory_controller --strict
```
