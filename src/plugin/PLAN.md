# microros_stm32h723_bridge 重构计划

> **状态**: 已确认 (v5 — 补充实时通信、关节映射与安全停机设计)
> **日期**: 2026-08-06
> **目标**: 将 plugin 重构为 C++ `hardware_interface` 插件,适配 ros2_control,**采用 MIT 阻抗控制方式**驱动电机,通过 micro-ROS 消息格式与 STM32H723 通信,串口传输由外部 `micro-ros-agent` 负责。
>
> **已确认决策**:
> 1. ✅ 插件端集成 **pinocchio** 计算重力补偿(前馈力矩 tau_ff)
> 2. ✅ 夹爪作为**第 7 关节**纳入插件,统一走 micro-ROS
> 3. ✅ 移除 `request_feedback` — micro-ROS pub/sub 架构下 STM32 持续 pub JointState,无需请求回传
> 4. ✅ 控制消息采用固定长度 `float32[7]`,关节顺序由协议固定
> 5. ✅ 归零在 deactivate 前由显式安全停机流程完成,固件 watchdog 负责失联降级

---

## 一、目标架构

```
MoveIt / JointTrajectoryController / JointStateBroadcaster
        │  ros2_control 框架 (read()/write() @ 控制周期)
        ▼
┌──────────────────────────────────────────────────────────┐
│ C++ SystemInterface 插件  microros_stm32h723_bridge             │
│  on_init: 读 URDF, 建 pub/sub + executor,加载 kp/kd 与 pinocchio 模型 │
│  read():  sub /stm32/joint_states → state_interface             │
│  write(): pinocchio 算重力补偿 tau_ff → 组装 MIT 参数 → pub      │
└──────────────────────────────────────────────────────────┘
        │  DDS (microros_stm32h723_bridge/JointCommand + sensor_msgs/JointState)
        ▼
┌──────────────────────────────────────────────────────────┐
│ micro-ros-agent  (serial --dev /dev/ttyACM0)                    │
└──────────────────────────────────────────────────────────┘
        │  XRCE-DDS over 串口
        ▼
   STM32H723 micro-ROS 客户端固件 (7 关节)
   (订阅 JointCommand → MIT 阻抗控制电机 / 持续发布 JointState 读编码器)
```

### 重力补偿数据流

```
read() 拿到 joint_states (7 关节位置)
        │
        ▼
pinocchio::computeGeneralizedGravity(q_model) → tau_model → 映射为 7 维 tau_hw
        │  (joint4 缩放 0.85, 沿用 follower_driver_node.py 约定)
        ▼
write(): tau_ff = tau_hw → 组装进 JointCommand.effort
```

### 核心原则

- **职责单一**: 插件不碰串口,只做 DDS 话题收发;串口传输完全交给 `micro-ros-agent`(外部进程)。
- **MIT 控制**: 上位机下发 (pos_des, vel_des, kp, kd, tau_ff),下位机执行 `tau = kp*(pos_des-pos) + kd*(vel_des-vel) + tau_ff`。
- **原生集成**: 作为 `pluginlib` 插件被 `ros2_control_node` 加载,与 MoveIt/控制器零额外接线。

---

## 二、MIT 阻抗控制设计

### 控制公式(下位机执行)

```
tau_motor = kp × (pos_des − pos_actual) + kd × (vel_des − vel_actual) + tau_ff
```

### 每关节参数(5)

| 参数 | 含义 | 单位 | 来源 |
|------|------|------|------|
| `position` | 目标位置 | rad (旋转) / m (夹爪) | controller command |
| `velocity` | 目标速度 | rad/s / m/s | controller command (默认 0) |
| `kp` | 位置增益 | 旋转关节 N·m/rad;直线关节 N/m | ROS 参数 (按状态切换) |
| `kd` | 速度增益 | 旋转关节 N·m·s/rad;直线关节 N·s/m | ROS 参数 (按状态切换) |
| `effort` | 前馈广义力 | 旋转关节 N·m;直线关节 N | 插件算重力补偿 (pinocchio) |

> **不包含 `request_feedback`**: micro-ROS pub/sub 架构下 STM32 持续 pub JointState,插件 sub 接收即可,无需请求回传机制。

### 消息格式

**标准 `sensor_msgs/JointState` 不够用** — 只有 position/velocity/effort,无法携带 kp/kd。使用自定义消息:

```
# microros_stm32h723_bridge/JointCommand.msg
std_msgs/Header header
uint32 sequence
uint8 mode              # HOLD=0, TRACK=1, RETURN=2, DISABLE=3
float32[7] position    # joint1-6(rad), gripper_finger_left_joint(m)
float32[7] velocity
float32[7] kp
float32[7] kd
float32[7] effort      # joint1-6(N·m), gripper(N)
```

另定义固定大小 `HardwareStatus.msg`: `header + last_command_sequence + mode + active_joint_mask + fault_code`,由 STM32 发布 `/stm32/status`,用于确认命令接收、已接入关节、DISABLE 完成及故障状态。

协议固定顺序为 `joint1`…`joint6`,`gripper_finger_left_joint`;不传字符串和无界序列,降低 MCU 动态内存及串口开销。STM32 必须检查 `sequence`/接收时间;命令超过 `command_timeout` 未更新时先 hold,超过 `disable_timeout` 后 disable。插件也校验状态消息的名称、长度、有限值和时间戳。

状态回传用标准 `sensor_msgs/JointState`(position/velocity/effort),兼容 JointStateBroadcaster。

### kp/kd 状态切换策略

按运行状态动态切换增益(沿用 [follower_driver_node.py](../../src/rebotarm_hw_driver/rebotarm_hw_driver/follower_driver_node.py) 的参数):

| 状态 | joint1-3 (大电机) kp / kd | joint4-6 (小电机) kp / kd | gripper (夹爪) kp / kd |
|------|--------------------------|--------------------------|----------------------|
| return (归零) | 30 / 3 | 15 / 2 | 6 / 1.5 |
| hold (保持) | 20 / 2 | 10 / 1.5 | 6 / 1.5 |
| track (跟踪) | 40 / 6 | 20 / 3 | 6 / 1.5 |
| idle (空闲) | 0 / 1 | 0 / 1 | 0 / 1 |

切换触发: activate 后进入 hold;收到有效控制器命令后进入 track;显式安全停机服务/动作进入 return。不得依赖 `on_deactivate()` 内部完成运动。

---

## 三、实施背景

全新创建 `ament_cmake` C++ 包,不参考 `src/plugin` 下任何历史代码。增益参数沿用 `src/rebotarm_hw_driver` 的现有约定(见上表)。

---

## 四、新包结构

```
src/plugin/microros_stm32h723_bridge/
├── package.xml                          # ament_cmake, depend: hardware_interface/rclcpp/sensor_msgs/pluginlib/pinocchio
├── CMakeLists.txt
├── microros_stm32h723_bridge.xml        # pluginlib 插件描述
├── msg/
│   ├── JointCommand.msg                 # 固定长度 MIT 控制消息
│   └── HardwareStatus.msg               # 模式确认、最后序号与故障码
├── include/microros_stm32h723_bridge/
│   ├── micro_ros_system.hpp             # SystemInterface 派生类
│   └── gravity_compensation.hpp        # pinocchio 重力补偿封装
├── src/
│   ├── micro_ros_system.cpp             # 插件实现 (init/read/write/activate)
│   ├── gravity_compensation.cpp        # pinocchio 模型加载 + computeGeneralizedGravity
│   └── plugin_export.cpp                # PLUGINLIB_EXPORT_CLASS
├── config/
│   ├── ros2_controllers.yaml            # controller_manager + JointTrajectoryController + JointStateBroadcaster
│   ├── mit_gains.yaml                   # kp/kd 按状态分组 (含 gripper)
│   └── micro_ros_system.yaml            # 插件参数 (topic 名/joint 名/超时/urdf 路径)
├── launch/
│   └── stm32h723_bridge.launch.py       # micro-ros-agent + ros2_control_node + spawn controllers
└── urdf/
    └── rebotarm.micro_ros.xacro         # ros2_control 块, plugin 指向本插件 (7 关节)
```

---

## 五、实施步骤

### 阶段 1 — 脚手架

1. 新建 `ament_cmake` 骨架(CMakeLists/package.xml/plugin xml/LICENSE)
2. 定义 `JointCommand.msg` 与 `HardwareStatus.msg`
3. 验证空插件可编译 + 消息可生成

### 阶段 2 — SystemInterface 核心 + 重力补偿

4. `MicroRosSystem : public hardware_interface::SystemInterface`
5. **`GravityCompensation` 类** (`gravity_compensation.hpp/.cpp`):
   - 加载固定基座 URDF,按关节名建立 hardware 索引到 Pinocchio `idx_q`/`idx_v` 的显式映射;禁止假定 `nq == nv == 7`
   - `compute(hw_q)` 将硬件状态映射到模型 q,调用 `computeGeneralizedGravity`,再映射回 7 个硬件关节
   - 校验 mimic/固定关节、关节方向、传动比及电机侧/关节侧力矩约定
   - joint4 缩放系数 0.85 (沿用 [follower_driver_node.py](../../src/rebotarm_hw_driver/rebotarm_hw_driver/follower_driver_node.py) 的 `_JOINT4_GRAVITY_SCALE`)
6. **`on_init()`**:
   - 读 URDF 参数(topic 名、joint 名[7个]、连接超时、urdf 路径)
   - 加载 `mit_gains.yaml` (kp/kd 按状态分组,含 gripper)
   - 初始化 `GravityCompensation` (加载 pinocchio 模型)
   - 创建独立 rclcpp 节点,pub `/stm32/joint_commands`、sub `/stm32/joint_states` 与 `/stm32/status`
   - 创建 `SingleThreadedExecutor` 后台线程;cleanup/shutdown 时 cancel、join,不得遗留线程
   - QoS 初始采用 `best_effort + keep_last(1)`,再按串口丢包测试调整
7. **`read()`**: 从 realtime-safe buffer 读取最新快照,按名称重排并校验长度、NaN、时间戳和超时;有效数据写入 7 维 state interface。超时返回 `ERROR`,由 controller manager 错误路径处理,同时依赖 STM32 watchdog 保底
8. **`write()`**:
   - 读 `hw_commands_position_` (7 维,来自控制器)
   - 调 `GravityCompensation::compute(current_pos)` → `tau_ff` (7 维)
   - 按 lifecycle 状态选 kp/kd (7 维)
   - 对目标位置、速度、增益和前馈广义力执行有限值检查与 URDF/配置限幅
   - 组装固定长度 `JointCommand` 并递增 sequence → 发布
9. **`on_activate()`**: executor 已运行后限时等待首帧有效状态;超时返回 `ERROR`;将 command 初始化为当前实测位置并进入 hold,避免零值跳变
10. **安全停机**: deactivate 前通过服务/动作启动限速归零状态机,持续 read/write,检查位置容差与超时;成功后发送 `mode=DISABLE`,等待 `HardwareStatus` 返回匹配 sequence 且 mode=DISABLE,再允许 deactivate。`on_deactivate()` 仅做最终状态检查和资源停用,不得阻塞运动
11. `PLUGINLIB_EXPORT_CLASS` 注册插件

### 阶段 3 — 集成配置

12. **URDF xacro**: `ros2_control` 块 `<plugin>microros_stm32h723_bridge/MicroRosSystem</plugin>`,7 关节接口 (joint1-6 + gripper),position command + position/velocity/effort state
13. **controller_manager yaml**: `JointTrajectoryController` + `JointStateBroadcaster`
14. **mit_gains.yaml**: 按 return/hold/track/idle 分组配置 kp/kd (含 gripper)
15. **launch**: 启动 `micro-ros-agent serial --dev ...` + `ros2_control_node` + spawn controllers

### 阶段 4 — 健壮性

16. 完成状态超时、固件 watchdog、命令新鲜度、关节限位、NaN、乱序/重复 sequence 与安全停机错误路径
17. 参数化串口/波特率走 agent 启动参数,插件只管 DDS 话题名;明确 agent 与固件发行版兼容矩阵

### 阶段 5 — 验证

18. `colcon build --packages-select microros_stm32h723_bridge`
19. mock 测试:一个 Python 节点模拟 STM32 发 JointState + 订阅 JointCommand,验证 MIT 参数 + 重力补偿力矩正确性
20. **首轮实机 Bring-up**:仅连接底座 `joint1`,固件固定 `active_joint_mask=0x01`,其余通道强制 DISABLE。先用受限测试节点完成话题通信、低增益小角度控制、超时 HOLD/DISABLE 和状态确认;此阶段不要求 7 关节 SystemInterface 激活
21. 通过单电机验收后再逐轴接入,最后执行完整 7 关节 ros2_control/MoveIt 联调

---

## 六、关键设计点

### 线程模型

- `read()/write()` 在 ros2_control 主循环**同步**调用
- rclcpp 话题回调**异步**
- executor 独占后台线程;订阅回调只校验并写入 realtime-safe buffer,不执行日志洪泛或阻塞操作
- `read()`/`write()` 不加长时锁、不分配无界容器;Pinocchio 数据在初始化阶段预分配

### command_interface 设计

ros2_control 标准 command_interface 只有 position/velocity/effort,无法直接表达 kp/kd。采用以下方案:

- **URDF 导出**: 每关节 `position` command_interface (标准,供 JointTrajectoryController 使用)
- **kp/kd 来源**: 通过 ROS 参数(`mit_gains.yaml`)配置,插件在 `write()` 时按 lifecycle 状态选取
- **tau_ff 来源**: 插件端 `GravityCompensation` 类调用 pinocchio 计算重力补偿

这样 JointTrajectoryController 仍输出 position,插件负责把 position + kp/kd + tau_g 组装成 MIT 指令。

### 重力补偿实现

- **类**: `GravityCompensation` (独立 hpp/cpp,便于测试)
- **模型来源**: 固定基座 URDF → `pinocchio::urdf::buildModel`;模型自由度数量不得硬编码
- **映射**: 按关节名和 `idx_q`/`idx_v` 在 ros2_control 7 维数组与 Pinocchio q/v/tau 间转换
- **计算**: `pinocchio::computeGeneralizedGravity(model, data, q)` 后仅提取对应的 7 个硬件关节
- **joint4 缩放**: `tau_g[3] *= 0.85` (沿用现有约定,减少 joint4 振荡)
- **夹爪广义力**: prismatic 关节单位为 N;若为 mimic/传动机构,必须通过实测决定是使用模型值还是明确置零
- **调用时机**: 每次 `write()` 时用最新 `read()` 拿到的位置计算

### 与现有 `rebotarm_hw_driver` 的关系

- 后者是 Python 独立节点(非插件),本插件是 C++ 标准 `hardware_interface`
- 两者**并存**,通过 URDF `ros2_control` 块的 `<plugin>` 切换
- MIT 参数与现有 [follower_driver_node.py](../../src/rebotarm_hw_driver/rebotarm_hw_driver/follower_driver_node.py) 中的常量保持一致,便于行为对齐

### 消息流

- **上 → 下**: MoveIt → JointTrajectoryController → 插件 `write()` 组装 JointCommand → pub `/stm32/joint_commands` → agent → STM32
- **下 → 上**: STM32 持续 pub JointState 与 HardwareStatus → agent → 插件 `read()` 更新接口/状态 → JointStateBroadcaster → `/joint_states`

### 关节接口(7 关节,沿用现有 rebotarm.ros2_control.xacro)

| 关节 | 类型 | command_interface | state_interface |
|------|------|------------------|----------------|
| joint1-6 | 旋转 (rad) | position | position + velocity + effort |
| gripper_finger_left_joint | 直线 (m) | position | position + velocity + effort |

---

## 七、依赖

### 编译时

- `ros2_control` (hardware_interface, controller_manager)
- `rclcpp_lifecycle`
- `pluginlib`
- `sensor_msgs`
- `rosidl_default_generators` (生成 JointCommand.msg)
- `rosidl_default_runtime`、`std_msgs` (消息运行时与 Header)
- `pinocchio` (重力补偿,需 `apt install ros-humble-pinocchio` 或源码编译)
- `ament_cmake` (buildtool)

### 运行时

- `micro_ros_agent`(外部进程,由 launch 启动)
- STM32H723 烧录 micro-ROS 客户端固件(需支持自定义 JointCommand/HardwareStatus 消息)

---

## 八、风险与待确认事项

1. **STM32 固件侧**: 假设已具备 micro-ROS 客户端,能 sub `JointCommand` + pub `JointState`/`HardwareStatus`。自定义 .msg 需固件端同步适配(运行 `micro_ros_setup` 的 mix 步骤纳入新消息)。**待确认固件是否支持自定义消息**。
2. ~~前馈力矩处理方式~~ → **已决定**: 插件端集成 pinocchio 计算重力补偿。
3. ~~夹爪处理~~ → **已决定**: 作为第 7 关节纳入插件,统一走 micro-ROS。
4. ~~request_feedback~~ → **已移除**: micro-ROS pub/sub 下 STM32 持续 pub,无需请求。
5. **micro-ros-agent 版本**: 需与 STM32 micro-ROS 客户端版本匹配。
6. **实时性**: `read()/write()` 中的 mutex 争用 + pinocchio 重力计算可能影响控制周期。若实测抖动大,可考虑: (a) lock-free 双缓冲; (b) 重力补偿降频计算 (如每 10 个周期算一次)。
7. **pinocchio 安装**: Humble 上 `ros-humble-pinocchio` 包可能不存在,需用 conda 或源码编译。需确认目标机器 (Jetson/Ubuntu) 的安装方式。
8. **安全协议**: 固件必须实现命令 watchdog、hold/disable 降级和 disable 确认;在协议确认前不得进行实机联调。
9. **接口包拆分**: 同包生成并使用消息可行,但 CMake 必须正确链接 rosidl typesupport。若 STM32 workspace 难以同步该包,将消息拆为独立 `microros_stm32h723_interfaces` 包。
10. **单电机测试边界**: 首轮只接 `joint1`;STM32 对 inactive joint 的非零命令必须拒绝。该阶段 `JointState` 只发布真实存在的 joint1,不得用零值伪造其余关节状态。

---

## 九、验证清单

- [ ] `colcon build --packages-select microros_stm32h723_bridge` 成功
- [ ] `ros2 interface show microros_stm32h723_bridge/JointCommand` 显示 7 维 MIT 参数 (不含 request_feedback)
- [ ] `ros2 control list_hardware_interfaces` 显示插件导出的 7 关节接口
- [ ] mock 节点发 JointState (7 关节),`ros2 topic echo /joint_states` 收到
- [ ] JointTrajectoryController 发指令,`ros2 topic echo /stm32/joint_commands` 收到含 kp/kd + 重力补偿力矩的 MIT 指令
- [ ] 重力补偿验证: 固定姿态下 `JointCommand.effort` 与映射后的 Pinocchio 理论广义力一致
- [ ] 全链路:控制器 → 插件 → agent(虚拟) → mock 下位机 → 状态回传 → `/joint_states`
- [ ] 单元测试覆盖关节重排、Pinocchio 索引映射、限幅、NaN、缺帧、乱序、超时与激活初值
- [ ] 断开 agent/串口后,STM32 在规定时间内 hold 并 disable
- [ ] 安全停机按限速轨迹归零,仅在位置容差满足并收到 disable 确认后 deactivate
- [ ] 单电机 Bring-up: `active_joint_mask=0x01`,只控制 joint1,完成通信、低增益小角度、超时与主动 DISABLE 测试
