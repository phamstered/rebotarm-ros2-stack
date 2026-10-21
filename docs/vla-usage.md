# VLA 使用指南

本页是 [VLA 集成 (scheme B)](./vla-integration) 的**实操版**：如何把外部 VLA 推理接到真机、如何用假源联调、如何安全地切换与停止。所有命令、参数均以当前 `src/plugin/microros_stm32h723_bridge` 代码为准。

::: tip 一句话
VLA 节点只发布 `/vla/joint_chunk`（`VlaJointChunk`，6 个关节角，rad）；`vla_position_controller` 负责限速与超时保持，**不要**直接写 `/stm32/joint_commands`。
:::

固件侧约束（MIT 字段校验、`COMMAND_TIMEOUT_MS`、fault 码）以 rebotarm-executor 的
[docs/vla.md](https://github.com/phamstered/rebotarm-executor/blob/main/docs/vla.md) 为准，本页只摘要与 ROS 侧相关的部分。

遥操 / VLA **数据采集** 的 stamp、`/stm32/status` 与 HOLD/fault 门禁见 [数据采集质量清单](./data-collection)。

## 架构

```
VLA 推理（Jetson / 外部主机，同一 ROS_DOMAIN_ID）
   │  microros_stm32h723_bridge/msg/VlaJointChunk
   ▼
/vla/joint_chunk
   │
   ▼
vla_position_controller            gravity_compensation_controller
  joint1–6/position（限速、超时保持）    joint1–6/effort（可选，重力前馈）
   │                                    │
   └──────────────┬─────────────────────┘
                  ▼
           MicroRosSystem.write()
   （关节限位钳位 + effort 限幅 → MIT JointCommand，100 Hz）
                  ▼
        /stm32/joint_commands → micro-ROS Agent → STM32H723 固件（MIT）
```

- `controller_manager` 的 `update_rate` 为 **100 Hz**（`config/arm_controllers.yaml`），即 `MicroRosSystem` 每 10 ms 发一帧 `JointCommand`。
- VLA 推理频率（建议 5–20 Hz）与 100 Hz 控制环解耦：控制器每个周期朝最新目标按限速逼近。

## 前置条件

1. **编译并 source overlay**（含 `VlaJointChunk` 消息与控制器插件）：

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select microros_stm32h723_bridge
   source install/setup.bash
   ```

2. **micro-ROS Agent active**：`systemctl status micro-ros-agent-stm32h723.service`；`ros2 topic echo /stm32/joint_states --once` 有数据。
3. **真机 bring-up**（见下节）：`ros2 control list_controllers` 中能看到 `vla_position_controller  ... inactive`。
4. **ROS_DOMAIN_ID 一致**：`start_arm_teleop.sh` 默认使用当前 `ROS_DOMAIN_ID`（未设置则 0），可用 `--ros-domain-id=N` 指定。
5. **外机推理**（推理不在 bring-up 主机上时）：
   - 与机械臂主机同一 `ROS_DOMAIN_ID`，且网络上 DDS 可互相发现（同网段 / 组播可达，或已配置的 DDS 发现方式）；
   - 外机也要编译并 source **含 `VlaJointChunk` 的 `microros_stm32h723_bridge` 包**（至少消息部分），否则无法 import / 发布该类型；
   - 验证：在外机 `ros2 topic list` 能看到 `/stm32/joint_states`，`ros2 interface show microros_stm32h723_bridge/msg/VlaJointChunk` 成功。

## 消息契约

`microros_stm32h723_bridge/msg/VlaJointChunk`，话题 **`/vla/joint_chunk`**（控制器参数 `chunk_topic`）：

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | `std_msgs/Header` | 建议填 `stamp`（发布时刻）与 `frame_id`（如模型名）。**控制器目前不读取 header**，超时按“收到时刻”计 |
| `positions` | `float32[]` | **绝对**关节角目标，单位 **rad**，顺序与控制器 `joints` 一致：`joint1, joint2, …, joint6`。长度必须为 6 |
| `timeout_sec` | `float32` | 本条消息的超时（秒）。`> 0` 且有限时覆盖 `command_timeout_sec`；`<= 0`（或 NaN）时使用参数默认 0.2 s |

控制器对消息的处理（`vla_position_controller.cpp`）：

- 长度 ≠ 6 → 整条忽略（节流告警 `expected 6 positions`）；
- 任一元素为 NaN/Inf → 整条忽略（节流告警 `non-finite position`），继续使用之前的目标；
- 合法消息只替换“待追目标”，实际输出每周期最多移动 `max_delta_per_sec × dt`。
- 多步 action chunk 需由发布端**自行降为当前一步**；本消息只承载一个关节向量。

关节限位由 `MicroRosSystem` 按 URDF（`urdf/rebotarm.arm.micro_ros.xacro`）钳位：joint1 ±2.8、joint2 [-3.14, 0]、joint3 [-3.14, 0]、joint4 [-1.87, 1.57]、joint5 ±1.57、joint6 ±3.14（rad）。超出范围不会报错，而是被静默钳位。

单条测试（CLI）：

```bash
ros2 topic pub --once /vla/joint_chunk microros_stm32h723_bridge/msg/VlaJointChunk \
  "{header: {frame_id: cli}, positions: [0.0, -0.3, -0.3, 0.0, 0.0, 0.0], timeout_sec: 0.0}"
```

::: warning `--once` 只会走一小段
限速 1 rad/s、超时 0.2 s：单条消息最多让每个关节移动约 **0.2 rad**，随后超时并**停在半路**。要真正到达目标，请持续发布，例如 `ros2 topic pub -r 10 ...`（Ctrl-C 停止后 0.2 s 内保持）。
:::

## 启动与切换

### 1. bring-up（`vla_position_controller` 自动加载为 inactive）

`launch/stm32h723_arm.launch.py` 在常规控制路径（`normal_control`）下以 `--inactive` 加载 `vla_position_controller`。推荐用不需要主臂的 trajectory 模式并开启重力补偿：

```bash
./scripts/start_arm_teleop.sh --position-controller=trajectory --gravity=true
# 或直接 launch：
ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py \
  position_controller:=trajectory enable_gravity_controller:=true
```

说明：

- `start_arm_teleop.sh` **没有** VLA 专用选项（`--position-controller` 仅支持 `teleop|trajectory|none`），VLA 一律通过 `switch_controllers` 切入。
- `position_controller:=none` 且开启重力/joint4 标定（重力整定路径）或 `state_only:=true` 时，**不会**加载 `vla_position_controller`。
- 若控制器未加载（例如旧版 launch），可手动加载：

  ```bash
  ros2 control load_controller --set-state inactive vla_position_controller
  ```

确认状态：

```bash
ros2 control list_controllers
# 期望：arm_trajectory_controller active、gravity_compensation_controller active、
#       vla_position_controller inactive、arm_teleop_controller inactive
```

### 2. 切到 VLA

`vla_position_controller` 与 `arm_teleop_controller` / `arm_trajectory_controller` 占用同一组 `joint*/position` 接口，**必须互斥**。重力补偿只占 `effort`，保持 active 即可。

```bash
# 当前是 trajectory active（上面的推荐启动方式）
ros2 control switch_controllers \
  --deactivate arm_trajectory_controller \
  --activate vla_position_controller \
  --strict

# 若当前是 teleop active
ros2 control switch_controllers \
  --deactivate arm_teleop_controller \
  --activate vla_position_controller \
  --strict
```

::: warning 始终使用 `--strict`，并按实际状态列控制器
推荐写法：

```bash
ros2 control switch_controllers \
  --activate vla_position_controller \
  --deactivate <当前 active 的 position 控制器> \
  --strict
```

- 同一条命令里 deactivate + activate 在同一个控制周期内完成，position 接口不会出现无人占用的空档；重力补偿（effort）不受影响、保持 active。
- Humble 的 `--strict` 下，只要 `--deactivate` 里有**本来就不是 active** 的控制器，或 `--activate` 里有**已经 active** 的控制器，整个请求就被拒绝、什么都不切换。所以：`--deactivate` 只写当前真正 active 的那**一个**（teleop **或** trajectory，先 `ros2 control list_controllers` 确认）；只有当 `gravity_compensation_controller` 确实是 inactive 时才把它加到 `--activate`。
- 注意 strict 只检查控制器**状态**；如果漏写了仍 active 的 position 控制器，请求本身可能返回成功，但 VLA 激活会因 `Resource conflict ... already claimed` 失败，切换后务必 `list_controllers` 确认。
:::

激活瞬间控制器读取当前关节位置作为目标并**原地保持**，直到收到第一条 chunk（日志：`holding current joint positions until a chunk arrives`）。

如重力补偿当前是 inactive，可在同一条命令里一并激活（若已 active 则**不要**写，否则 strict 会拒绝整个请求）：

```bash
ros2 control switch_controllers \
  --activate vla_position_controller gravity_compensation_controller \
  --deactivate arm_trajectory_controller \
  --strict
```

### 3. 停止 / 切回

```bash
# 先停推理节点（控制器会在超时后保持最后位置），再切回 trajectory
ros2 control switch_controllers \
  --deactivate vla_position_controller \
  --activate arm_trajectory_controller \
  --strict
```

- 切回 `arm_teleop_controller` 前要确认主臂与 `joint_state_relay` 已在运行，否则 teleop 会把从臂拉向缺省/陈旧目标；最稳妥是重新运行 `start_arm_teleop.sh`（默认 teleop）。
- 需要整机安全下电：`ros2 service call /microros_stm32h723_hardware/safe_stop std_srvs/srv/Trigger {}`（RETURN 回零后 DISABLE，见 [控制栈](./controllers)）。

## 参数（`vla_position_controller`）

| 参数 | 默认（代码 / `arm_controllers.yaml`） | 说明 |
|------|------|------|
| `joints` | `[joint1 … joint6]` | 占用的 position 接口，也是 `positions` 的顺序 |
| `chunk_topic` | `/vla/joint_chunk` | 订阅话题（SystemDefaults QoS：reliable、keep last 10） |
| `command_timeout_sec` | `0.2` | 无新 chunk 超过该时长即保持；必须 > 0 且有限，否则 configure 失败 |
| `max_delta_per_sec` | `1.0` | 每关节限速（rad/s）；必须 > 0 且有限 |

修改：编辑 `config/arm_controllers.yaml` 后重新 bring-up（参数在 `on_configure` 读取，运行中 `ros2 param set` 不会生效）。

## 假源联调（mock publisher）

`mock_vla_chunk_publisher.py` 发布正弦关节角：`positions[i] = amplitude_rad · sin(2π·frequency_hz·t + 0.35·i)`。

| 参数 | 默认 |
|------|------|
| `topic` | `/vla/joint_chunk` |
| `publish_rate_hz` | `10.0` |
| `amplitude_rad` | `0.15` |
| `frequency_hz` | `0.1` |
| `timeout_sec` | `0.0`（使用控制器默认 0.2 s） |
| `num_joints` | `6` |

::: danger 正弦以 0 rad 为中心（绝对角）
mock 的目标围绕**零位**摆动，而不是围绕当前姿态。如果激活时臂不在零位附近，控制器会以最高 1 rad/s 把各关节拉向 0 附近。另外 joint2 / joint3 上限为 0，正半周会被钳在 0。首次联调请先把臂放到零位附近并托住。
:::

推荐顺序：

1. **HOLD 验证（不发 chunk）**：只激活 `vla_position_controller`，不启动任何发布者 → 臂应原地保持；`/stm32/joint_commands` 持续 100 Hz、位置不变。
2. **小幅度**（臂在零位附近、手托住）：

   ```bash
   ros2 run microros_stm32h723_bridge mock_vla_chunk_publisher.py \
     --ros-args -p amplitude_rad:=0.05 -p frequency_hz:=0.1
   ```

3. 确认无误后再用默认 `amplitude_rad:=0.15`，最后接真实策略。

观察：

```bash
ros2 topic hz /vla/joint_chunk                    # ≈ publish_rate_hz
ros2 topic echo /vla/joint_chunk --once
ros2 topic hz /stm32/joint_commands               # ≈ 100 Hz
ros2 topic echo /stm32/joint_commands --once      # mode=1(TRACK)、position 随 chunk 变化
ros2 topic echo /stm32/joint_states --once        # 实测位置跟随
ros2 topic echo /stm32/status --once              # fault_code 应为 0
```

注意：`/stm32/joint_commands` 中的 `position` 是电机侧原始值（经 `position_scales`/`position_offsets` 换算），可能与 rad 关节角不完全相同。

## Python 接入样例

最小 rclpy 节点：订阅 `/joint_states`（`joint_state_broadcaster`）获取当前 q，按固定频率调用 `policy(q)`，把输出 q[6] 发布为 `VlaJointChunk`。不依赖任何模型库，`policy` 换成你的推理即可。

```python
#!/usr/bin/env python3
"""Minimal VLA bridge: policy(q) -> VlaJointChunk on /vla/joint_chunk."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from microros_stm32h723_bridge.msg import VlaJointChunk

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def policy(q):
    """Replace with model inference. Must return 6 absolute joint angles in rad."""
    return list(q)  # hold current pose


class VlaBridge(Node):
    def __init__(self):
        super().__init__('vla_bridge')
        self.declare_parameter('rate_hz', 10.0)        # 5–20 Hz recommended
        self.declare_parameter('timeout_sec', 0.0)     # <= 0: controller default (0.2 s)
        self._q = None
        self._pub = self.create_publisher(VlaJointChunk, '/vla/joint_chunk', 10)
        self.create_subscription(JointState, '/joint_states', self._on_state, 10)
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

    def _on_state(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if all(j in index for j in JOINTS):
            self._q = [msg.position[index[j]] for j in JOINTS]

    def _tick(self):
        if self._q is None:
            return  # no state yet: publish nothing, controller keeps holding
        target = [float(x) for x in policy(self._q)]
        if len(target) != len(JOINTS) or not all(math.isfinite(x) for x in target):
            self.get_logger().warn('policy output rejected', throttle_duration_sec=1.0)
            return
        msg = VlaJointChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'my_policy'
        msg.positions = target
        msg.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = VlaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

要点：

- 推理慢于 `1 / command_timeout_sec`（默认 5 Hz）时，臂会在两次推理之间“停—走”；可适当增大消息里的 `timeout_sec`，但它没有上限，过大意味着推理挂掉后控制器会**长时间继续追最后一个目标**。
- 推理阻塞时不要在同一回调里阻塞 `spin`；重模型建议放到独立线程/进程，节点只负责发布最新结果。

## 安全注意

**ROS 侧（本仓库代码的实际行为）**

- **互斥**：同一时间只能有一个 position 控制器（`vla_position_controller` / `arm_teleop_controller` / `arm_trajectory_controller`）active；请用单条 `switch_controllers ... --strict` 完成 deactivate + activate（见上文）。
- **超时**：超过有效超时（消息 `timeout_sec` 或 `command_timeout_sec`=0.2 s）没有新 chunk，控制器**保持最后一次已输出的位置**（不是最后的目标，而是限速后实际走到的位置），只打印一次 `VlaJointChunk timed out` 警告。新 chunk 到来后从该位置继续限速逼近。
- **超时期间仍是 TRACK**：`MicroRosSystem` 继续以 100 Hz 发送 `mode=TRACK` 的保持帧（重力补偿若 active 也照常叠加）。控制器日志里提到的 “Upper layer should keep MODE_HOLD” 只是提示，**代码不会主动切换到 HOLD**。
- **NaN / 长度错误**：整条 chunk 被丢弃，等同于没收到，最终同样走超时保持。
- **限速**：`max_delta_per_sec` 默认 1.0 rad/s，对每个关节单独钳位（单周期 dt 上限 0.1 s）。
- **限位 / 力矩**：`MicroRosSystem` 把位置钳到 URDF 限位，把 effort 钳到增益文件的 `effort_limits`（`arm_mit_gains.yaml`：j1–3 10 Nm、j4–6 6 Nm）；`gravity_compensation_controller` 自身 `effort_limits` 为 10 Nm。

**固件侧（摘要，详见 [rebotarm-executor docs/vla.md](https://github.com/phamstered/rebotarm-executor/blob/main/docs/vla.md)）**

- 固件在 **TRACK** 下 100 ms 无合法 `JointCommand` → HOLD（`User/motor_bridge.c` 只对 TRACK 做这一步；RETURN 下没有 100 ms HOLD，直接走 500 ms 锁存）。只有 ros2_control / Agent / USB 链路**整体停发**时才会触发；VLA 推理慢**不会**触发（控制器仍在发保持帧）。
- 500 ms 无命令 → 锁存 **fault 11（CMD_LOSS）**，之后拒绝所有命令，**只能重启固件恢复**。
- 任一关节 `|effort| > 10 Nm` → 固件**拒收整帧**（fault 3），不是钳位；kp/kd 必须 ≥ 0。
- 固件 HOLD 下 effort = 0、**无重力补偿**，长时间 HOLD 时重关节（joint2/3）可能下垂。

**首次实机清单**

1. 臂周围清空，有人手托/随时可按急停；Agent active，`/stm32/status` `fault_code == 0`。
2. bring-up：`--position-controller=trajectory --gravity=true`，确认 `vla_position_controller` 为 inactive。
3. 把臂放到零位附近（若用 mock），激活 VLA，**先不发 chunk**，确认原地保持。
4. mock `amplitude_rad:=0.05` → 观察 `/stm32/joint_states` 跟随、`fault_code` 保持 0。
5. 停 mock，确认 0.2 s 后保持；切回 trajectory，确认无跳变。
6. 再接真实策略，先用低频（5 Hz）、小动作任务；全程监看 `/stm32/status`。

## 与 rebotarm-debug-desk 配合

desk 侧操作详见 **[rebotarm-debug-desk · VLA 使用指南](https://phamstered.github.io/rebotarm-debug-desk/vla)**（源文件 [docs/vla.md](https://github.com/phamstered/rebotarm-debug-desk/blob/main/docs/vla.md)；草稿中：[PR #102](https://github.com/phamstered/rebotarm-debug-desk/pull/102)，合并前以 PR 内容为准）。概要：

- Status 页 VLA 卡片的一键启停，本质上就是上面的 `switch_controllers` 调用（重力补偿保持不变）。
- Monitor 页的 “+VLA” 预设用于同时观察 `/vla/joint_chunk` 与 `/stm32/*` 话题。

::: danger 不要在 desk 一键 Teleop/Trajectory 拉起 ros2_control 时点 desk “Start VLA”
在 desk 修复之前：如果 ros2_control 是由 desk 的一键 Teleop/Trajectory（即 `start_arm_teleop.sh`）**拉起的**，**不要**点 desk 的 “Start VLA”。它会先停止该脚本，而**停止脚本会把 ros2_control 一起关掉**。

`scripts/start_arm_teleop.sh` 的实际退出流程（`trap cleanup EXIT INT TERM`）：仅当 ros2_control 是**本脚本启动的**时，`cleanup` 才会：

1. 调用 `/microros_stm32h723_hardware/safe_stop`（`timeout 8`）→ `MicroRosSystem` 进入 RETURN，约 3 s（`return_duration_ms`）内把各关节插值回 **零位**，到位（`zero_tolerance` 0.03）后发 DISABLE（RETURN kp ≠ 0 的增益下不会稳定停在 DISABLE，见下）；
2. 每秒查一次 `/stm32/status`，**最多等 10 s** 等待 `mode == 3`（DISABLE）；
3. 再停掉 ros2_control 进程组（SIGINT，约 3 s 后 SIGTERM，再 1 s 后 SIGKILL）及残留的 launch / `ros2_control_node` / `robot_state_publisher`。

desk 停止脚本的方式是 `kill -INT -$pid`，约 5 s 后 `kill -TERM -$pid`（发给脚本所在**进程组**）。ros2_control 由脚本用 `setsid` 启动、位于独立会话，**不会**直接收到 desk 的信号，只会被脚本的 `cleanup` 停掉。按目前的代码阅读（**未经实机验证**），可能出现两种结果：

关键点（代码阅读）：`safe_stop` 在 `MicroRosSystem` 里是一个**锁存标志**（`micro_ros_system.cpp:413` 置位，只在 hardware `on_activate` 时清除，`:222`）。服务一旦调用成功，回零由 ros2_control 进程里的 `write()` 执行（`:280-301`），**杀掉脚本并不会打断回零**；只要 ros2_control 还活着，之后所有 position 目标（包括 VLA）都会被回零/DISABLE 逻辑覆盖。因此 `switch_controllers` 可以返回成功、`vla_position_controller` 也显示 active，但**臂不会跟随 VLA**。

可能的结果：

- **结果 A：ros2_control 仍在运行**（cleanup 在 safe_stop 之后、停 ros2_control 之前被 SIGTERM 打断）→ 臂**意外回到零位**，之后 VLA 无效。`/stm32/status` 的表现取决于增益文件：
  - `gravity_mit_gains.yaml`（RETURN kp 全 0）：每帧都发 DISABLE 且复用同一个 sequence，固件按过期拒收 → **稳定显示 `mode=3`、`fault_code=4`**；DISABLE 下不触发 500 ms 锁存，**不会**出 fault 11。
  - `arm_mit_gains.yaml` / `arm_gripper_mit_gains.yaml`（RETURN kp ≠ 0，即 VLA 推荐路径用的增益）：发出一帧 DISABLE 后，下一周期 `write()` 会因 `command_mode_ != RETURN` 重新进入 RETURN（`:287-290`），于是**在零位附近以 RETURN 增益保持**，约每 3 s 再发一帧过期的 DISABLE。`/stm32/status` 多数时间显示 `mode=2`、`fault_code=0`，偶尔闪 `fault_code=4`，**很少能看到 `mode=3`**。（疑似 `MicroRosSystem` 的 bug，见下方说明。）
- **结果 B：ros2_control 已退出**（safe_stop 后 cleanup 完成了进程停止，或进程自行退出）→ `switch_controllers` 直接失败（没有 controller manager）。固件侧：若退出时固件已处于 DISABLE，不会锁存；若处于 RETURN / TRACK（上面第二种增益下几乎总是 RETURN），500 ms 后锁存 **fault 11**，切 HOLD（effort = 0，无重力补偿，重关节可能下垂），之后拒绝所有新命令，只能重启 MCU（见 [rebotarm-executor docs/vla.md](https://github.com/phamstered/rebotarm-executor/blob/main/docs/vla.md)）。

**如果已经点了**，按顺序排查：

1. `ros2 topic echo /stm32/status --once`（多看几次）：`mode`（2 = RETURN，3 = DISABLE）与 `fault_code`；
2. `ros2 control list_controllers`：controller manager 是否还在；
3. 按下表处理，然后按下面的手动路径重新启动：

| 现象 | 含义 | 处理 |
|------|------|------|
| `mode=3` + `fault_code=4`，controller manager 在 | safe_stop 已执行，ros2_control 仍在发过期 DISABLE | **重启 MCU 没用**（锁存在 ros2_control 里）。重启 ros2_control（固件处于 DISABLE，停发不会触发 fault 11）；或尝试重新激活硬件（见下） |
| `mode=2`（偶尔 `fault_code=4`），臂停在零位附近，controller manager 在 | safe_stop 已执行，处于 RETURN 循环 | 同样**重启 MCU 没用**；需要重启 ros2_control。停掉时固件处于 RETURN，预计 500 ms 后锁存 fault 11 → 再按下一行处理 |
| `fault_code=11` | 命令丢失锁存 | 先把臂安全托回零位，再**重启 MCU**（锁存不会自行清除） |

重新激活硬件（不重启 ros2_control，**未经实机验证**）：Humble 支持 `ros2 control set_hardware_component_state <name> inactive|active`，硬件组件名为 **`Stm32H723ArmSystem`**。`on_activate` 会清除 safe_stop 锁存、把 `disable_sequence_` 归零并从固件上报的 `last_command_sequence` 续号（`micro_ros_system.cpp:220-224`），逻辑上是干净的。但 `on_deactivate` 只有在固件确认 `mode == DISABLE` 且 `last_command_sequence == disable_sequence_` 时才放行（`:233-238`）：`mode=3 + fault 4` 的情况满足，RETURN 循环的情况**不满足**（会被拒绝）。建议先 deactivate 所有 command 控制器再操作：

```bash
ros2 control switch_controllers --deactivate vla_position_controller gravity_compensation_controller --strict
ros2 control set_hardware_component_state Stm32H723ArmSystem inactive
ros2 control set_hardware_component_state Stm32H723ArmSystem active
ros2 control switch_controllers --activate arm_trajectory_controller gravity_compensation_controller --strict
```

**推荐做法**：在 Jetson 终端手动启动

```bash
./scripts/start_arm_teleop.sh --position-controller=trajectory --gravity=true
```

然后用**一条** strict 切换进入 VLA（position 接口无空档、重力补偿保持 active）：

```bash
ros2 control switch_controllers \
  --activate vla_position_controller \
  --deactivate arm_trajectory_controller \
  --strict
```

desk 侧修复正在进行中：当脚本拥有 ros2_control 时，desk 不再停止脚本；无法判断时拒绝操作；切换后校验 `vla_position_controller` 确为 active。
:::

::: tip Monitor 解码 VlaJointChunk
Monitor 要解码 `VlaJointChunk`，desk Bridge 的运行环境必须 source 本仓库含该消息的 `microros_stm32h723_bridge` 包；否则 `/vla/joint_chunk` 的 Hz / 曲线为空，但 VLA 启停（`switch_controllers`）仍可正常使用。
:::

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| `switch_controllers` 失败：controller not found / not loaded | 走了重力整定路径（`position_controller:=none` + gravity/标定）或 `state_only`，launch 未加载 VLA | 用常规路径重启，或 `ros2 control load_controller --set-state inactive vla_position_controller` |
| 切换后 VLA 未 active，日志 `Resource conflict ... already claimed` | teleop / trajectory 仍 active，position 接口被占用 | 在同一条命令里 `--deactivate` 当前 active 的 position 控制器 |
| `switch_controllers --strict` 失败：`can not be deactivated since it is not active` / `since it is already active` | `--deactivate` 里有本就 inactive 的控制器，或 `--activate` 里有已 active 的（如重力补偿） | 先 `list_controllers`，按实际状态只列需要变化的控制器，保留 `--strict` |
| `ros2 topic hz /vla/joint_chunk` 没数据 | 外机没装/没 source 含 `VlaJointChunk` 的包；`ROS_DOMAIN_ID` 不同；DDS 不可达 | 外机 `ros2 interface show microros_stm32h723_bridge/msg/VlaJointChunk`；核对 domain；外机能否看到 `/stm32/joint_states` |
| chunk 有 Hz，臂不动 | `vla_position_controller` 未 active；teleop/trajectory 仍占接口；消息长度≠6 或含 NaN（看控制器日志） | `list_controllers`；看 `Ignoring VlaJointChunk` 警告 |
| 臂“走一下停一下” | 推理频率 < 5 Hz 导致频繁超时 | 提高推理频率，或适当增大 `timeout_sec` |
| 臂动得很慢 / 跟不上 | `max_delta_per_sec` 限速（默认 1 rad/s） | 确认安全后在 YAML 中调大并重启 |
| 某关节到不了目标 | 目标超出 URDF 限位被钳位（如 joint2/3 > 0） | 检查目标范围 |
| `/stm32/status` fault 3 | 某关节 effort > 10 Nm，整帧被拒 | 检查重力参数 / 增益文件 `effort_limits` |
| `mode=3` + fault 4（或 `mode=2` 停在零位不动） | `safe_stop` 已执行且 ros2_control 仍在运行，position 目标被回零逻辑覆盖 | 重启 MCU 无效；重启 ros2_control 或重新激活 `Stm32H723ArmSystem`（见上文 desk 一节） |
| fault 11 | ros2_control 或 Agent 在非 DISABLE 状态下停发 > 500 ms | 先安全回零，再重启 MCU；见固件文档 |

## 限制与后续

- 首版只接受 **关节角**（joint1–6），无速度/力矩前馈。
- **不控制夹爪**：`gripper_teleop_controller` 独立；VLA 不占用夹爪接口。
- 无末端位姿 / IK 接口。
- 每条消息只有一个目标向量，不做 chunk 内插值；`header.stamp` 暂未使用。
- 后续（P1）：基于 `arm_trajectory_controller` / trajectory 的时间参数化执行 action chunk。
