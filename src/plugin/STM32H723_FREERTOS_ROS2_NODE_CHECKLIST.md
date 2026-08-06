# STM32H723 FreeRTOS micro-ROS 节点实现清单

> 对应上位机设计：[PLAN.md](./PLAN.md)。目标是让 STM32H723 作为 micro-ROS 客户端，通过串口连接 `micro-ros-agent`，执行 7 关节 MIT 控制并持续反馈状态。

> **首轮 Bring-up 范围**：初次功能测试只连接机械臂底座电机 `joint1`。该阶段仅验证 STM32 ↔ micro-ROS Agent ↔ ROS 2 的通信、状态反馈和单电机 MIT 控制；其余 6 路必须保持 DISABLE。完整 7 关节 ros2_control 联调放在后续阶段。

## 1. 节点与 ROS 实体

- [ ] 创建单一节点：`stm32h723_motor_bridge`。
- [ ] 创建订阅者 `/stm32/joint_commands`，类型 `microros_stm32h723_bridge/msg/JointCommand`。
- [ ] 创建发布者 `/stm32/joint_states`，类型 `sensor_msgs/msg/JointState`。
- [ ] 创建发布者 `/stm32/status`，类型 `microros_stm32h723_bridge/msg/HardwareStatus`。
- [ ] 使用 `rclc_executor` 管理订阅和 ROS 定时器；executor handle 数量必须与实体数一致。
- [ ] 使用 `best_effort`、`keep_last(1)` 作为控制和状态话题的初始 QoS；经串口丢包测试后再调整。
- [ ] 启动时执行 agent ping；断线后销毁 ROS 实体并按有限退避时间重建，禁止紧密重连循环。
- [ ] 若需要 ROS 时间戳，连接后同步 agent epoch；同步失败时状态中明确时间无效，安全逻辑始终使用本地单调时钟。

## 2. 固定通信协议

`JointCommand` 固定关节顺序：

```text
[joint1, joint2, joint3, joint4, joint5, joint6,
 gripper_finger_left_joint]
```

- [ ] 接收 `sequence`、`mode`、`position[7]`、`velocity[7]`、`kp[7]`、`kd[7]`、`effort[7]`。
- [ ] 支持 `HOLD=0`、`TRACK=1`、`RETURN=2`、`DISABLE=3`；未知模式立即拒绝并上报故障。
- [ ] 拒绝 NaN/Inf、越界位置、负增益、超限速度或广义力。
- [ ] 使用 wrap-safe 的 `uint32_t` 序号比较，忽略重复或过期命令。
- [ ] `HardwareStatus` 回传最后接受的 command sequence、实际模式和 fault code。
- [ ] `HardwareStatus` 回传 `active_joint_mask`;首轮单电机测试固定为 `0x01` (`joint1`)。
- [ ] `JointState` 数组按 `active_joint_mask` 在初始化时一次性分配（首轮 1 个，完整系统 7 个）；循环内不得重新分配。
- [ ] 明确夹爪接口单位：position=m、velocity=m/s、effort=N；其余关节使用 rad、rad/s、N·m。

## 3. FreeRTOS 任务划分

| 任务 | 建议职责 | 原则 |
|---|---|---|
| `motor_control_task` | 固定周期读取编码器、执行 MIT 公式、写 CAN/驱动器 | 最高优先级；不得调用 ROS API |
| `safety_task` | watchdog、限位、过流/过温、急停和 disable | 高优先级；可直接覆盖控制输出 |
| `micro_ros_task` | executor spin、发布状态、agent 重连 | 不得阻塞控制环 |
| `diagnostic_task` | 低频统计、LED/调试输出 | 最低优先级；生产模式限制日志 |

- [ ] 使用静态任务、静态队列/缓冲区，并记录每个任务的 stack high-water mark。
- [ ] ROS 回调只验证并复制命令到双缓冲区，然后原子切换；不得在回调内操作 CAN 或等待 mutex。
- [ ] 控制任务读取一致快照；状态发布读取另一份快照，避免 ROS 和控制任务互相阻塞。
- [ ] 使用硬件定时器或稳定的 `vTaskDelayUntil()` 驱动控制周期，并统计 deadline miss、最大抖动和执行时间。

## 4. MIT 控制与状态机

- [ ] 每周期执行 `tau = kp*(q_des-q) + kd*(dq_des-dq) + effort_ff`。
- [ ] 在应用前执行关节方向、零点、传动比及关节侧↔电机侧力矩换算。
- [ ] 对最终电机位置、速度、力矩和力矩变化率进行二次限幅；上位机限幅不能替代固件保护。
- [ ] `HOLD` 首次进入时锁存当前位置为目标，不使用全零目标。
- [ ] `TRACK` 仅接受持续更新且有效的命令。
- [ ] `RETURN` 执行上位机给出的限速轨迹，不在固件中瞬间跳到零位。
- [ ] `DISABLE` 将 kp/effort 清零，按驱动器要求安全撤销使能，并在完成后发布确认。
- [ ] 故障具有明确的锁存/清除规则；急停、驱动器故障和传感器无效必须优先于 ROS 命令。

## 5. Watchdog 与失联策略

- [ ] `command_timeout` 到期：从 TRACK 进入 HOLD，目标锁存为当前实测位置。
- [ ] `disable_timeout` 到期：进入 DISABLE；该定时器使用本地硬件/RTOS 时钟。
- [ ] agent 断线、串口错误或 executor 错误不得阻止 watchdog 运行。
- [ ] 编码器过期、CAN 节点掉线或反馈非有限值时立即禁止对应电机输出，并上报 fault code。
- [ ] 上电默认 DISABLE；只有通信就绪、反馈有效且收到合法模式命令后才允许使能。

## 6. 初始化与清理顺序

1. 初始化时钟、MPU/cache、内存池、UART DMA、CAN 和编码器。
2. 创建控制与安全任务，保持电机 DISABLE。
3. 配置 micro-ROS 串口 transport、allocator、support、node、pub/sub 和 executor。
4. ping agent 并同步时间；发布启动状态。
5. 收到首个合法命令后按状态机决定 HOLD/TRACK。
6. 重连或销毁时先进入 DISABLE，再按 executor → pub/sub → node → support 的逆序释放实体。

## 7. 验收清单

### 7.1 首轮单电机 Bring-up

- [ ] 固件使用编译期或只读配置 `active_joint_mask=0x01`;不得通过普通 ROS 控制消息临时启用未接电机。
- [ ] 仅初始化并使能底座电机总线地址;joint2-6 和夹爪保持 DISABLE,收到对应非零命令时拒绝并上报 `INACTIVE_JOINT_COMMAND`。
- [ ] `/stm32/joint_states` 在此阶段只发布 `joint1` 的 name/position/velocity/effort,不为未接关节伪造反馈。
- [ ] 首轮采用 ROS 话题级测试节点或 CLI 发送固定长度 `JointCommand`;暂不要求完整 7 关节 hardware plugin 激活。
- [ ] 首先保持电机悬空或拆除负载,设置低 kp/kd、零前馈力矩和严格的速度/力矩限幅。
- [ ] 依次验证：Agent 发现节点 → 状态持续回传 → HOLD 无跳变 → 小角度 TRACK → command timeout HOLD → disable timeout DISABLE → 主动 DISABLE 确认。
- [ ] 目标运动限制为底座安全小角度范围;测试现场保留物理急停,首次测试不得直接发送归零或大角度轨迹。
- [ ] 记录命令 sequence、往返延迟、状态频率、串口丢包、控制周期抖动、目标/实际位置和 fault code。

示例检查命令：

```bash
ros2 node list
ros2 topic hz /stm32/joint_states
ros2 topic echo /stm32/status
ros2 topic echo /stm32/joint_states
```

固定数组消息不适合手工反复输入。建议提供受限的 `single_motor_smoke_test.py`,只允许写 `position[0]`,并把其余目标、增益和 effort 强制置零。

### 7.2 完整系统验收

- [ ] Agent 启停 100 次无死锁、内存持续增长或任务泄漏。
- [ ] 断开串口后在配置时限内依次进入 HOLD 和 DISABLE。
- [ ] 重复、乱序、NaN、越界及未知模式命令均被拒绝并产生正确 fault code。
- [ ] 7 关节反馈顺序、单位、符号和时间戳与上位机一致。
- [ ] 固定姿态下 MIT 输出及前馈广义力与离线计算一致。
- [ ] 最大控制周期抖动、deadline miss、栈余量和串口带宽均有记录。
- [ ] 后续实机测试按“joint1 单电机 → 逐轴接入 → 六轴 → 夹爪 → 完整 ros2_control”逐级进行，并保留物理急停。

## 参考

- [micro-ROS 官方文档](https://micro.ros.org/docs/)
- [micro-ROS FreeRTOS 应用教程](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)
- [rclc Executor 说明](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/executor/)
