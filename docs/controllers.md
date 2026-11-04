# 控制栈与后端

从臂有 **三条硬件/仿真后端**；其上是共享的 ros2_control 控制器概念（或 CAN 路径上的等价逻辑）。选后端时不要对同一物理臂混开两条命令通路。

## 后端总览

| 后端 | 入口包 | 命令如何到达电机 / 仿真 | 典型启动 |
|------|--------|-------------------------|----------|
| **Gazebo** | `rebotarm_gazebo` | `JointTrajectoryController` → `ign_ros2_control` | `./scripts/start_gazebo.sh` |
| **CAN MIT** | `rebotarm_hw_driver` | `/joint_commands`（JointState）→ Python MIT 环（含重力） | `./scripts/start_jetson.sh --mode=real` |
| **micro-ROS** | `microros_stm32h723_bridge` | ros2_control `MicroRosSystem` → `/stm32/joint_commands` → agent → STM32 | `stm32h723_arm.launch.py` / `./scripts/start_arm_teleop.sh` |

关节中继只是命令源适配器，配置见：

- Gazebo：`joint_state_relay/config/relay.yaml`（`mode: gazebo`）
- CAN：`relay_real.yaml`（`mode: real`）
- micro-ROS teleop：`relay_microros_arm.yaml`（`mode: ros2_control`）

## Gazebo 路径

1. Ubuntu：`start_gazebo.sh`（`ROS_DOMAIN_ID=1`，ign gazebo 6 + controller_manager）
2. 可选 Jetson：`start_jetson.sh --mode=gazebo` + domain bridge + relay
3. 或纯规划：`ros2 launch rebotarm_moveit_config gazebo_demo.launch.py`

控制器（`rebotarm_gazebo/config/ros2_controllers.yaml`）：`arm_controller`、`gripper_controller`、`joint_state_broadcaster`。

约定：用 `ign gazebo`（非 `gz sim`）；模型 spawn 用 `ign service`；`prepare_urdf.py` 处理 `package://`。

## CAN 路径（`rebotarm_hw_driver`）

- **不是** ros2_control 插件：节点直接连 `RobotArm` / `Gripper`（`reBotArm_control_py`）
- 订阅 `/joint_commands`，发布 `joint_states`
- 控制环内：RETURN → HOLD → TRACK / IDLE；Pinocchio 风格重力（joint4 有缩放）；夹爪 m↔电机 rad
- 总线：`can0` @ 1 Mbps（`start_jetson.sh --mode=real` 会尝试拉起）

适合沿用现有 CAN 从臂、暂不接入 `MicroRosSystem` 的部署。

## micro-ROS 路径（推荐统一 ros2_control）

```
命令源（MoveIt / teleop / trajectory / 标定）
        │
        ▼
controller_manager @ 100 Hz
  position → arm_teleop_controller | arm_trajectory_controller
  effort   → gravity_compensation_controller | joint4_calibration_controller
        │
        ▼
MicroRosSystem.read/write
  合成 MIT：position + velocity（仅轨迹接口持有且 TRACK）+ kp/kd(模式) + effort
        │  JointCommand / JointState / HardwareStatus
        ▼
micro-ros-agent (serial, 常由 systemd 托管)
        ▼
STM32H723 固件（rebotarm-executor）
```

### 控制器矩阵（`arm_controllers.yaml`）

| 控制器 | 类型 | 接口 | 默认 |
|--------|------|------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | state pos/vel/eff | 激活（正常控制） |
| `arm_teleop_controller` | JointGroupPositionController | cmd position · j1–6 | 默认激活 |
| `arm_trajectory_controller` | JointTrajectoryController | cmd position + velocity · 允许 partial goal | 默认可 inactive |
| `gripper_teleop_controller` | JointGroupPositionController | 夹爪 position | `enable_gripper:=true` 且 teleop 时激活 |
| `gripper_trajectory_controller` | JointTrajectoryController | 夹爪 position + velocity | `enable_gripper:=true` 且 trajectory 时激活；与夹爪 teleop 互斥 |
| `gravity_compensation_controller` | 自定义 | **仅** effort | 可选 |
| `joint4_calibration_controller` | 自定义 | effort + 多 Trigger 服务 | 标定模式；与重力互斥 |
| `vla_position_controller` | 自定义 | **仅** position · j1–6 | VLA scheme B；与 teleop/trajectory 互斥；可与重力并行 |

### JTC 状态发布目标

真实 `arm_controllers.yaml` 的 `arm_trajectory_controller` 与
`gripper_trajectory_controller` 均将 `state_publish_rate` 设为 **100 Hz**。
旧 micro-ROS `ros2_controllers.yaml` 的 arm/gripper 控制器以及 MoveIt 参考配置
的 arm 控制器同步使用该目标；Gazebo 仍保持独立的 500 Hz 控制环与 200 Hz 状态发布目标。

`controller_manager.update_rate` 仍为 **100 Hz**。状态发布率只控制
`<controller>/controller_state` 的发送节奏，不会提高控制环频率、STM32 采样率或
`/joint_states` 的发布率，也不改变 `action_monitor_rate`、增益、约束或运动限值。
100 Hz 是配置目标，实际接收率可能因周期量化、调度、实时发布锁竞争及 DDS 传输而降低，
需要分别核对运行参数与带时间戳的实测频率；仅修改文件不会更新已配置的控制器。

提高发布率可以减少异步反馈比较中的等待，但不能保证两条反馈在同一时刻采样。
仍需按时间戳对齐数据，并保留现有一致性算法、误差门限与安全门。
生产重新配置应走受控 Controller 生命周期与既有安全流程，不直接重启维持输出的控制栈。

### 安全与运维要点

- 串口只由 **一个** agent 占用；launch 默认 `start_agent:=false`
- deactivate 前：`ros2 service call /microros_stm32h723_hardware/safe_stop std_srvs/srv/Trigger {}`
- 单电机冒烟：固件 mask `0x01` + `single_motor_smoke_test.py`（不要直接上七关节插件）
- 纯重力调参：`--position-controller=none --gravity=true`
- Joint4 标定：`--mode=calibration`；服务 `begin_pose` / `capture_pose` / `release_pose` / `fit` / `reset`
- MIT 增益文件：默认 `arm_mit_gains.yaml`（RETURN/HOLD/TRACK/IDLE）。`--gripper=true` 改用 `arm_gripper_mit_gains.yaml`（夹爪 RETURN kp=0，effort 限幅为 5/3/2 N·m）。纯重力或 Joint4 标定且无位置控制器时用 `gravity_mit_gains.yaml`（各模式 kp=0）
- 硬件不叠加轨迹积分。effort 只在增益文件限幅内转发；重力由重力控制器负责

包内说明：`src/plugin/microros_stm32h723_bridge/README.md`。

## MoveIt 如何接到后端

`rebotarm_moveit_config` 的 `moveit_controllers.yaml` 把规划结果交给 **`arm_trajectory_controller`** 的 `FollowJointTrajectory`。这与 Gazebo / MoveIt demo 里的 **`arm_controller`** 不是同一个名字。

- **demo**：mock hardware（`mock_components/GenericSystem`），其 `ros2_controllers.yaml` 仍定义 `arm_controller`
- **gazebo_demo**：Gazebo 的 `arm_controller` / `gripper_controller`（控制环 500 Hz，状态发布目标 200 Hz）
- **真机 micro-ROS**：`real_move_group.launch.py` 只启动 `move_group`。默认执行 `arm_trajectory_controller/follow_joint_trajectory`；`enable_gripper:=true` 时再加入已激活的 `gripper_trajectory_controller`。它不 spawn、不切换控制器
- 六轴规划上限 **2 rad/s、2 rad/s²**，默认速度/加速度缩放 **1.0**；夹爪 **0.02 m/s、0.04 m/s²**。这是规划上限，不改固件限幅、MIT 增益或重力补偿。起点容差 0.01 rad
- SRDF 组：`arm`（六轴 IK 与 HOME）、`gripper`（关节目标）、`arm_with_gripper`（七关节关节目标，或 `link6` 位姿加夹爪关节）。没有单独的七轴 IK；Humble 按关节集合包含关系复用 `arm` 的 KDL

controller 与 hardware **无显式绑定**，靠关节名 + 接口类型匹配，见 [ros2_control 关联机制](./ros2_control_association)。

## 如何选择

| 目标 | 建议后端 |
|------|----------|
| 仿真轨迹 / 可视化 / MoveIt 联调 | Gazebo（± MoveIt） |
| 快速验证规划配置、无硬件 | MoveIt `demo.launch.py` |
| 统一 ros2_control、重力、标定、与固件协议 | **micro-ROS 桥** |
| 现有 CAN 栈、Python MIT 环 | `rebotarm_hw_driver` |
| 主臂带动从臂 | 任一后端 + `joint_state_relay` 对应 mode |

## 主从遥操（应用层之一）

遥操不是独立「产品模式」，而是：

1. 主臂发布 `joint_states`（可选 `/leader` namespace）
2. `joint_state_relay` 做映射、死区、限速、超时
3. 写入上述某一后端的命令接口

micro-ROS 一键守卫启动：`./scripts/start_arm_teleop.sh`（九段自检：主臂、agent、STM32 mask、controllers、hardware、反馈、relay、命令序列 ack）。可同时 `--gravity=true`、`--gripper=true`，或切换 `--position-controller=trajectory`。

## VLA（scheme B）

外部推理发布 `/vla/joint_chunk`（`VlaJointChunk`）→ `VlaPositionController` 写 position，可选 `gravity_compensation_controller` 写 effort → `MicroRosSystem` 合成 MIT。不要把直接写 `/stm32/joint_commands` 作为主路径。详见 [VLA integration](./vla-integration) 与 [VLA 使用指南](./vla-usage)。



### RETURN 收敛与 supervisor 退出

六轴与带夹爪的 arm MIT profile 将 joint4 RETURN Kp 设为 45，Kd 保持 2；TRACK/HOLD 增益、effort limits 与 `zero_tolerance=0.03 rad` 均不变。此次修正针对当前负载在 HOME 附近的静态残差：Jetson / ROS_DOMAIN_ID=0 真机上，Kp=15 时 joint4 约停在 −0.0494 rad，Kp=30 时约 −0.0349 rad，均不能完成正常 RETURN；Kp=45 时约 −0.0242 rad，正常 RETURN→DISABLE 成功。后续 Teleop↔Trajectory 启停也通过。此结果不代表全工作空间或不同负载已验证，也不替代重力模型标定。

`start_arm_teleop.sh` 的退出 trap 只停止本次创建的 relay、leader 与控制器进程组，不再按全局可执行文件名称清理其他实例。启动时仅复用当前 ROS 域的 ControllerManager，不能把隔离域或其他机器人域的实例当作本域服务。safe-stop 检查当前 ROS 域，退出时还核对控制器所属启动组，并要求新鲜 HardwareStatus 的 `mode=DISABLE, fault_code=0` 后才关闭控制器。桌面端已经执行同域安全清理后使用 `--clean=false`；直接命令行 `--clean=true` 的历史启动清理行为仍存在。


重复验收边界：随后的一次正式路径 RETURN 在 joint3≈−0.03185 rad 停住（fault=0）。实验将 joint3 RETURN Kp 从 30 改为 45、75 后，位置仍固定约 −0.03376 rad，虽然反馈力矩随之变化；这两项实验未作为默认配置发布，joint3 保持 Kp=30。已明确区分正常 RETURN 成功与现场授权的 DISABLE 恢复，不能把后者当作回零通过。此状态需要确认 joint3 机械限位/接触与零位，未确认前保留 0.03 rad 门禁、停止进一步增加驱动力。该现场问题单独跟踪于 #41。

### safe_stop 之后的模式

`/microros_stm32h723_hardware/safe_stop` 只置位锁存，由 `write()` 执行回零。任一已配置关节的 RETURN kp 非 0 时，先在 `return_duration_ms`（3000）内插向零位；相关关节进入 `zero_tolerance`（0.03 rad）后进入 DISABLE。夹爪 RETURN kp 为 0，不要求夹爪回到 0。`gravity_mit_gains.yaml` 的 RETURN kp 全 0，safe_stop 直接 DISABLE。

DISABLE 一旦发出就保持到下一次 hardware `on_activate`：每帧序号递增，位置、速度、kp、kd、effort 为 0。当前实现不会在 DISABLE 后再跳回 RETURN，也没有轨迹积分项。未进容差时 RETURN 保持挂起，`start_arm_teleop.sh` 的退出清理不会在缺少 `mode=DISABLE, fault_code=0` 时拆掉控制器。现场被托住的臂可以另调 `/microros_stm32h723_hardware/disable` 直接释放输出；这不是回零成功。


## 拖动示教与独立夹爪

`./scripts/start_arm_teleop.sh --mode=teaching --clean=false` 使用专用
`teaching_mit_gains.yaml`：六轴所有模式 Kp=0、Kd=1，重力 effort 控制器 active，
独立 `gripper_trajectory_controller` 控制标定夹爪。必须先按现有安全流程停止旧控制栈；
该模式拒绝复用既有 ControllerManager。仅在现场支撑且操作区域安全时启动。

硬件以只读 `drag_teaching` 参数标识已验证的专用配置；拒绝六轴 position/velocity
接口认领，避免拖动时切回刚性跟踪。夹爪沿用现有夹爪 HOLD/TRACK 增益及 effort 限幅。
六轴前馈限幅沿用纯重力模式的每轴 ±10 N·m，夹爪单独保持 ±2 N·m。
不能复用普通位置模式的六轴 5/3 N·m 限幅：零 Kp 下，重力前馈被截断后没有位置反馈力矩补足。
例如已观察姿态的 Joint3 模型需求为 −7.718 N·m，旧示教配置只发送 −5 N·m；
修复后保留该前馈需求，同时仍执行上述限幅。此限幅是软件前馈边界，不代表新增额定负载能力。
启动初始化为当前实测位置，不自动闭爪。该模式不是力控夹持，也不自动录制示教轨迹。

配套桌面通过已安装的 `teaching_gripper.py` 发送单关节 FollowJointTrajectory。
每个操作使用独立 UUID；目标为 0–0.05 m 的单指行程，5 秒三次轨迹满足既有
20 mm/s、40 mm/s² 上限。每次校验专用模式、重力/夹爪控制器、无六轴位置所有者、
新鲜米制反馈和七轴无故障 TRACK；执行中继续检查反馈与控制器。取消仅作用于同一 UUID。
动作成功后须确认夹爪连续 200 ms 停稳；不要求正在手动拖动的六轴停稳。
完成不表示夹持成功。请求超时或反馈失效时请求取消，无法核验时明确报告停止未确认。

RETURN 增益全部为零，所以 `safe_stop` 在此模式直接进入 DISABLE，**不会主动回零**。
退出或切换前必须托住机械臂及物体；夹爪也会失去电机输出。模式/夹爪请求共用
桌面 `arm-mode.lock`，夹爪运动期间先取消并等待结束，再关闭控制环境。
手动拖动没有运动规划器避碰，重力模型与负载变化可能造成漂移。

验证入口：`test_teaching.py`、硬件 gtest `DragTeaching` 和隔离域
`test/smoke_teaching.py`（模拟 MCU）。这些不代表真实机械臂拖动手感、夹持力或负载验证。
