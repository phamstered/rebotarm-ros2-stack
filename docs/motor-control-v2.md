# Motor Control V2 与旧 MIT 栈

本包提供 POS_VEL 手动控制所需的独立 ROS 消息/服务，以及旧 MIT 控制栈的模式准入门禁。
`MicroRosSystem`、Teleop、JTC、VLA、示教和重力辨识继续发布 V1 MIT，不发布或订阅 V2 运动命令。
软件接口和模拟测试不等于现场电机已经支持 POS_VEL；实际支持掩码由固件逐轴验收后报告。

## 冻结的 V1 与独立 V2

`JointCommand.msg` 和 `HardwareStatus.msg` 布局不变。V1 的 `mode=0/1/2/3` 仍表示
HOLD/TRACK/RETURN/DISABLE。驱动模式不追加到 V1。

| 接口 | 默认名称 | 内容 |
| --- | --- | --- |
| V1 `JointCommand` | `/stm32/joint_commands` | MIT；`mode=3` 为所有驱动模式的全局停止入口 |
| V2 `JointCommandV2` | `/stm32/joint_commands_v2` | 仅 POS_VEL；HOLD/TRACK，7 轴位置和逐轴正值速度上限 |
| `MotorControlStatus` | `/stm32/motor_control_status` | 运行身份、实际模式读回、选中/使能/验证/支持掩码、事务、拒绝原因和故障 |
| `MotorControl` 服务 | `/stm32/motor_control` | CONFIGURE / ACQUIRE / RELEASE / CANCEL |

自定义 V1 `command_topic` 时，V2 命令名为 `command_topic + "_v2"`，状态和服务名取
其父命名空间加 `/motor_control_status`、`/motor_control`。例如模拟器默认使用
`/posvel_test/joint_commands`，不会连接生产 `/stm32` 端点。

V2 用 `boot_id + control_epoch + source_id + lease_id` 约束周期帧；每帧有共享的
`sequence`、固定 `active_joint_mask`、`control_mode=2`、`float32[7] position/velocity_limit`。
未选轴参数必须为零且保持失能。速度上限必须为正且在固件已核验上限内；不复用 MIT
有符号速度、不带 Kp/Kd/Effort，也不支持 RETURN。所有浮点数必须在转为 float32 后仍有限。
HOLD 帧的全部 `position[]` 必须为零，固件锁存新鲜实测位置并对实测位置检查限位；
不把零字段当成回零目标，也不要求该轴行程包含零。HOLD 的速度上限仍须为正且有效。

服务 action 是 1=CONFIGURE、2=ACQUIRE、3=RELEASE、4=CANCEL。请求包含 operation ID、
boot/config 身份、source、mask、驱动模式、lease 和最后一帧 DISABLE sequence。
CONFIGURE 的 `accepted=true` 只是接受事务；必须等待状态中的相同 operation ID 达到
OP_SUCCESS，且各所选轴 `verified_joint_mask/control_modes` 读回一致。输出关闭证据必须来自
实际 DISABLE 后的新鲜反馈：`disabled_verified=true` 表示全 7 轴的实际 DISABLED 反馈均新鲜，
且接收时间晚于本次 CAN stop 发出；`enabled_joint_mask=0` 本身不构成证据。
启动默认 `last_command_sequence=0` 不算证据；序列自然回绕到 0 则合法。

错误码 0–13 分别为 NONE、INVALID、STALE_IDENTITY、NOT_DISABLED、BUSY、UNSUPPORTED、
MODE、LEASE、SEQUENCE、NONFINITE、LIMITS、FAULT、READBACK、TIMEOUT。拒绝诊断
`rejected_sequence/reject_code` 与锁存 `fault_code` 分开，不把丢包或错模式当作已成功执行。
DISABLE 撤销 lease，故障 11 下仍可请求关闭输出，但不能清除锁存故障。500 ms watchdog
不能解释为一定关闭输出；停止结果仍以反馈为准。
ACQUIRE 后仍处于 DISABLE、且尚未收到第一帧合法 V2 时，lease 有 500 ms 握手窗口；
第一帧受理后使用 100 ms 指令超时（HOLD 也需要持续刷新）。空闲 lease 到期不会使能输出。

## MIT 生命周期准入

`MicroRosSystem` 在 hardware 激活、控制器认领接口和每次写入前检查：

- 已发现 V2 状态发布者或收到过 V2 状态后，本实例永久要求新鲜的 V2 状态，不因发布者消失而回退。
- 状态必须有协议 2、非零 boot ID、递增的源时间、未回退的配置代次；所有必需轴均已验证为 MIT。
- 已退休 boot 的迟到反馈不能重新打开门禁。单个 hardware 实例最多记忆 16 次启动变更，
  超出后保持门禁关闭，须重新构建该实例；不会为释放有界历史而重新接受旧 boot。
- 不允许切换中、未完成配置、故障、配置错误、V2 source 归属或 lease 尚未释放。
- 写入过程中准入失效或 MCU 重启会锁存 DISABLE，不经过 RETURN，也不在状态恢复后自动继续旧目标。
  下一次合法 hardware 生命周期才能重新激活。停止帧清零所有控制字段，并重新分配 sequence；
  MCU 重启后依据新 boot 状态重建停止 sequence，不沿用旧 boot 的确认。

旧固件没有 V2 状态时，默认保留原 MIT 行为和已有反馈/关节 mask 安全门。这是有意的兼容策略，
不是“没有状态就证明实际 MIT”；新固件仍需在 MCU 上独立拒绝错模式 V1。
已升级部署可设置 hardware 参数 `require_motor_control_status=true`，从启动时就禁止旧固件回退。
两份 Xacro 及 `stm32h723_arm.launch.py` / `stm32h723_bridge.launch.py` 均提供同名参数；
自定义硬件描述还可设置：

| hardware 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `require_motor_control_status` | `false` | 是否即使从未见过 V2，也要求其读回状态 |
| `motor_control_status_topic` | command topic 父命名空间下的状态名 | 隔离命名空间或自定义状态端点 |
| `motor_control_status_timeout_ms` | `1000` | 状态接收有效期，允许 1–1000 ms；重复源时间不会刷新有效期 |

恢复旧控制栈之前，先完成全局 DISABLE，再确认所需轴回到 MIT、无切换、owner 和 lease 为零。
旧的 safe-stop/deactivate 仍要求 `HardwareStatus` 无故障 DISABLE；本次没有把故障清理等同于停机确认。

## 无硬件验证

`motor_control_sim.py` 是纯标准库、外部注入时间的状态模型；`mock_motor_control_v2.py`
把它接到真实 ROS 类型与服务。模拟器的速度/位置限值和 joint1–6 支持掩码只是测试数据，
不用于真实设备能力认证；没有 CAN、串口或真实电机输出。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select microros_stm32h723_bridge
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=219 ros2 run microros_stm32h723_bridge mock_motor_control_v2.py
```

自动测试覆盖异步配置与读回失败、重复 operation、身份/lease/sequence 竞争、未选轴、
float32 溢出、速度与位置约束、HOLD、watchdog、故障锁存和停止后迟到帧。
`colcon test` 另外在隔离 ROS 域 219 验证服务生成、序列化、V1 抢占拒绝及 V1 全局停止；
纯 C++ 门禁测试覆盖旧固件回退、状态过期/重放、配置回退、实际驱动和 lease。
硬件生命周期 gtest 验证模式丢失后 DISABLE 锁存、无效目标不阻断停止和重启 sequence 重建。

```bash
colcon test --packages-select microros_stm32h723_bridge --event-handlers console_direct+
colcon test-result --verbose
```

寄存器能否在失能时写入、POS_VEL HOLD 的速度/方向/单位、真实 watchdog 和输出关闭效果，
仍需 executor 按实际电机型号验收。没有这些证据时固件应保持该轴 POS_VEL 能力关闭。
