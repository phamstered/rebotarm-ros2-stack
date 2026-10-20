# VLA integration (scheme B)

> 实操步骤（启动/切换、假源联调、Python 接入样例、安全清单、FAQ）见 **[VLA 使用指南](./vla-usage)**。
> Firmware-side constraints: [rebotarm-executor docs/vla.md](https://github.com/phamstered/rebotarm-executor/blob/main/docs/vla.md).

Vision-Language-Action (VLA) inference drives the arm through ros2_control
**position** interfaces, not by writing `/stm32/joint_commands` directly.

## Path

```
External VLA / inference node
        │  microros_stm32h723_bridge/msg/VlaJointChunk
        ▼
 /vla/joint_chunk
        │
        ▼
VlaPositionController          gravity_compensation_controller
  (claims joint*/position)       (claims joint*/effort, optional)
        │                         │
        └──────────┬──────────────┘
                   ▼
            MicroRosSystem
         (MIT synthesis → JointCommand)
                   ▼
         /stm32/joint_commands → agent → STM32 firmware
```

Firmware is unchanged. There is no end-effector / IK layer in this path.

## Controllers

| Controller | Interfaces | Notes |
|------------|------------|-------|
| `vla_position_controller` | **position** j1–6 | Subscribes to chunks; rate-limits; holds on timeout/NaN |
| `gravity_compensation_controller` | **effort** j1–6 | Safe to run in parallel |
| `arm_teleop_controller` | position j1–6 | **Mutually exclusive** with VLA |
| `arm_trajectory_controller` | position j1–6 | **Mutually exclusive** with VLA |

Same position command interfaces cannot be claimed by two active controllers.
Documented in `config/arm_controllers.yaml`.

## Message

`VlaJointChunk.msg`:

- `std_msgs/Header header`
- `float32[] positions` — length must match the controller `joints` list (6)
- `float32 timeout_sec` — if `> 0`, overrides the controller timeout for that chunk

Topic: **`/vla/joint_chunk`** (parameter `chunk_topic`).

## Parameters (`vla_position_controller`)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `joints` | joint1–6 | Claimed position interfaces |
| `chunk_topic` | `/vla/joint_chunk` | Subscription |
| `command_timeout_sec` | `0.2` | Hold last command if no fresh chunk |
| `max_delta_per_sec` | `1.0` | Per-joint rate limit (rad/s) |

On chunk timeout or rejected NaN input the controller **holds the last
commanded (rate-limited) positions**. Note: MicroRosSystem keeps sending
`MODE_TRACK` hold frames at 100 Hz; nothing switches the host to `MODE_HOLD`
automatically (the controller log line is advisory). Firmware HOLD only
triggers if commands stop entirely (100 ms). Do not treat a direct write to `/stm32/joint_commands` as
the primary VLA path.

## Activate

`stm32h723_arm.launch.py` loads `vla_position_controller` **inactive** on
normal bring-up (`normal_control`). Activate via `switch_controllers`
(teleop/trajectory must be inactive); no separate `load_controller` needed:

Use a single atomic `--strict` switch (deactivate + activate happen in the
same control cycle, so the position interfaces are never left unclaimed, and
gravity stays active):

```bash
# Example after stm32h723_arm.launch.py with teleop already running:
ros2 control switch_controllers \
  --activate vla_position_controller \
  --deactivate arm_teleop_controller \
  --strict
```

Humble `--strict` rejects the whole request if any controller in
`--deactivate` is not active, or any controller in `--activate` is already
active. So list only the one position controller that is currently active
(teleop **or** trajectory), and add `gravity_compensation_controller` to
`--activate` only if it is actually inactive. Strict checks controller states
only: if you forget an active position controller, VLA activation fails with
`Resource conflict ... already claimed`, so check `ros2 control list_controllers`
afterwards.

If the controller is not loaded (e.g. gravity-tuning launch path), use
`ros2 control load_controller --set-state inactive vla_position_controller`.

See [VLA 使用指南](./vla-usage) for the full procedure and the desk one-click
script caveat.

## Mock publisher

Safe sine-wave chunks for bring-up without a real policy:

```bash
source install/setup.bash
ros2 run microros_stm32h723_bridge mock_vla_chunk_publisher.py
```

Optional parameters: `amplitude_rad` (default 0.15), `frequency_hz` (0.1),
`publish_rate_hz` (10), `topic`, `num_joints`.

## Out of scope

- End-to-end CI with a real VLA model
- Firmware changes
- EE Cartesian / IK commanding
- Gripper VLA control (optional later; controller claims arm joints only today)
