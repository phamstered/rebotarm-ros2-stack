# microros_stm32h723_bridge

ROS 2 Humble `hardware_interface::SystemInterface` for a seven-joint STM32H723 micro-ROS motor controller. The plugin receives `JointState`, computes Pinocchio gravity feed-forward, and publishes fixed-size MIT commands. Serial transport remains owned by `micro-ros-agent`.

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

## Single-joint ros2_control

After the topic-level smoke test passes, start the dedicated joint1 profile while
the systemd Agent is running:

```bash
ros2 launch microros_stm32h723_bridge stm32h723_single_joint.launch.py
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

This profile requires firmware `active_joint_mask=0x01`, exports only
`joint1/position`, accepts a one-name `JointState`, disables gravity
compensation, and writes zeros to all six inactive protocol slots. It must not
be used if any other motor channel is enabled. The profile uses the dedicated
`single_joint_mit_gains.yaml` limits: HOLD kp/kd 2.0/0.2, TRACK 5.0/0.5,
joint1 effort limit 1.0, and zero gains/effort for every inactive slot.

## Single-joint teleoperation

The single-joint launch loads `joint1_teleop_controller` in the inactive state
while keeping `joint1_controller` active for trajectory tests. Start the relay,
then switch the command-interface owner explicitly:

```bash
ros2 launch joint_state_relay relay_to_microros_single_joint.launch.py
ros2 control switch_controllers \
  --deactivate joint1_controller \
  --activate joint1_teleop_controller --strict
```

The relay consumes `/leader/joint_states` and streams a one-element
`Float64MultiArray` to `/joint1_teleop_controller/commands`. Relative alignment
captures both arms' positions on the first fresh sample, the command rate is
limited to 0.3 rad/s, and a leader timeout holds the last follower target. Move
only joint1 during this profile and keep a physical emergency stop available.

Return to trajectory control with:

```bash
ros2 control switch_controllers \
  --deactivate joint1_teleop_controller \
  --activate joint1_controller --strict
```
