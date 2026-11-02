# VLA / 遥操数据采集质量清单

本页是 **录制前 / 录制中** 的质量门禁清单，面向 LeRobot 等下游数据集。栈侧提供 stamp 预检脚本与文档；Desk UI 负责 Episode 开始门禁与提示；LeRobot 丢弃规则由 Desk / 采集适配层实现（本仓库不包含 `rebotarm_vla.lerobot_recorder`）。

相关文档：[VLA 使用指南](./vla-usage)、[VLA 集成](./vla-integration)、[构建与运行](./build-run)。固件与 micro-ROS Agent 时间同步细节由 **rebotarm-executor** 文档补充。录制脚本的硬门禁仍是「`/stm32/joint_states` stamp 非零」；TF 用的反馈时钟比这一条更严，见下文「反馈时钟」。

::: tip 一句话
开录前跑 `./scripts/check_collection_ready.sh`：`header.stamp` 非零、`/stm32/status` 的 `fault_code==0` 且 `mode==TRACK(1)`；始终同时录 status；靠近 HOLD / fault 11 不要录。
:::

## 1. Agent 时间 / 非零 stamp

`/stm32/joint_states` 由 MCU 经 **micro-ROS Agent** 发布（`MicroRosSystem` 只 **订阅**，不发布该话题）。stamp 仅在 Agent 的 epoch/时间可用时有效；否则为 `sec=0, nanosec=0`。

录制前必须确认 stamp **非零**：

```bash
# 硬门禁（失败 exit 1，适合 Desk SSH / 录制前）
./scripts/check_collection_ready.sh

# 可选：指定 domain / 超时
./scripts/check_collection_ready.sh --ros-domain-id=0 --timeout=15
```

手工抽查：

```bash
ros2 topic echo /stm32/joint_states --once
ros2 topic echo /stm32/status --once
# 期望 header.stamp.sec / nanosec 不同时为 0
```

`scripts/start_arm_teleop.sh` 在自检通过后会以 `--warn-only` 调用同一脚本（不阻断遥操 bring-up）；**开录前请再跑一次硬门禁**。

Agent epoch / 时间如何配置，见 executor 侧文档。`check_collection_ready.sh` 在本栈内只强制 **stamp 非零**，外加 status 的 TRACK 与 `fault_code==0`。

### 反馈时钟（TF / Robot View，不是录制脚本）

`stm32_state_republisher.py` 在转发之前用 `FeedbackClock` 检查**原始** stamp，不用本机收到时间冒充源时间。拒绝：零或非法 stamp、超前超过 50 ms、陈旧达到 1 s、相对上一帧回拨超过 50 ms。重复或轻微乱序的样本直接丢掉。相对单调时钟，ROS 时间跳变超过 250 ms 会重新准入。恢复要求连续 200 ms 的有效递增反馈。静默 1 s 记为过期。

固件诊断 v1（16 字）仍兼容。v2（24 字）还要求固件状态已同步、上次同步年龄 &lt; 15 s、诊断在 3 s 内收到；固件 epoch 变化会重新等待。每个 republisher 在 `~/time_sync_status` 发布只读 JSON（`state`、`generation`、`source_offset_ms`、`feedback_age_ms`）。链路通不等于时间已同步。

通过准入的 `/robot_feedback/joint_states` 保持源频率和源 stamp。只有 `robot_state_publisher` 使用另外的 `/robot_feedback/tf_joint_states`，按单调时钟限到 **20 Hz**。RSP 的 `ignore_timestamp=true` 只绕过它自己的旧 epoch 节流。采集预检通过，不表示 TF 时钟已是 `synchronized`。

## 2. 始终录 `/stm32/status`，并丢弃坏 Episode

除关节与相机外，**务必同步录制** `/stm32/status`（`microros_stm32h723_bridge/msg/HardwareStatus`）：

| 字段 | 含义 | 采集侧建议 |
|------|------|------------|
| `header.stamp` | 状态时间戳 | 非零；可与关节帧对齐 |
| `mode` | `MODE_HOLD=0` / `MODE_TRACK=1` / `MODE_RETURN=2` / `MODE_DISABLE=3` | 有效片段应为 **TRACK (1)** |
| `fault_code` | 固件故障码；0 = 无故障 | 有效片段应为 **0** |
| `last_command_sequence` | MCU 已确认的命令序号 | 用于发现 **序号停滞**（链路卡死） |
| `active_joint_mask` | 已接入关节位掩码 | 臂 `0x3F`，含夹爪 `0x7F` |

丢弃（或 Desk 提示后人工丢弃）Episode 的条件：

- `fault_code != 0`
- `mode != TRACK`（1）
- `last_command_sequence` 在录制窗口内长时间不递增（sequence stall）

Desk 当前约定：Episode **开始** 用 stamp / HOLD / fault 门禁；**中途不自动丢弃**，仅提示。LeRobot 侧的自动 drop 规则由 Desk 实现，不在本 PR。

## 3. 不要在 HOLD / fault 11 附近录制

`fault_code=11`（`COMMAND_LOSS_LATCHED`）表示命令丢失锁存：固件侧 effort 被清零，臂会下垂（sag）。`MODE_HOLD` 同样不应作为采集有效段。

- 预检或 `ros2 topic echo /stm32/status` 看到 `fault_code=11` 或 `mode=0`：**不要开录**。
- 锁存清除需按安全流程复位 MCU / 走 safe_stop 与 bring-up（见 [VLA 使用指南](./vla-usage) 故障表）。栈侧 `safe_stop` 相关修复（如 PR #20）有助于避免误入 fault 11，但与本清单的录制门禁是分开的工作。

## 4. action = 命令，observation = 关节状态（摘要）

下游数据集约定（LeRobot / VLA 训练）：

| 角色 | 推荐来源 | 说明 |
|------|----------|------|
| **observation** | `/stm32/joint_states`（或 ros2_control `/joint_states`） | 实际测得的关节位置等 |
| **action** | 实际下发的命令（如 `/arm_teleop_controller/commands`、夹爪 commands，或等价的 `/stm32/joint_commands` 合成结果） | **不要**把观测关节角当作 action |

本页不展开 schema；Desk / `rebotarm_vla` 采集器负责字段映射。栈侧只保证上述话题在遥操路径上可用，并提供 stamp/status 预检。

## 预检脚本行为摘要

`scripts/check_collection_ready.sh`：

1. 各 echo 一帧 `/stm32/joint_states`、`/stm32/status`（可配置超时）。
2. 任一话题 `header.stamp` 为 `sec==0 && nanosec==0` → **失败**。
3. `fault_code != 0` 或 `mode != 1`（TRACK）→ **失败**。
4. 打印 `last_command_sequence`（若存在）。
5. 默认硬失败 `exit 1`；`--warn-only` 仅告警并 `exit 0`（供 `start_arm_teleop.sh`）。

不依赖、也不配置 Agent epoch；只检查已发布消息是否满足录制条件。
