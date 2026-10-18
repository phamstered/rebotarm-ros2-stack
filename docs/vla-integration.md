# VLA integration (scheme B)

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

On chunk timeout or rejected NaN input the controller **holds the last valid
command positions**. Upper layer / firmware mode should remain **HOLD** while
inference is stale. Do not treat a direct write to `/stm32/joint_commands` as
the primary VLA path.

## Activate

`stm32h723_arm.launch.py` loads `vla_position_controller` **inactive** on
normal bring-up (`normal_control`). Activate via `switch_controllers`
(teleop/trajectory must be inactive); no separate `load_controller` needed:

```bash
# Example after stm32h723_arm.launch.py with teleop already running:
ros2 control switch_controllers \
  --deactivate arm_teleop_controller \
  --activate vla_position_controller \
  --strict
```

Keep gravity on if desired:

```bash
ros2 control switch_controllers --activate gravity_compensation_controller
```

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
