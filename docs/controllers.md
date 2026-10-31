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
| `arm_trajectory_controller` | JointTrajectoryController | cmd position · 允许 partial goal | 默认可 inactive |
| `gripper_teleop_controller` | JointGroupPositionController | 夹爪 position | `enable_gripper:=true` 时 |
| `gravity_compensation_controller` | 自定义 | **仅** effort | 可选 |
| `joint4_calibration_controller` | 自定义 | effort + 多 Trigger 服务 | 标定模式；与重力互斥 |
| `vla_position_controller` | 自定义 | **仅** position · j1–6 | VLA scheme B；与 teleop/trajectory 互斥；可与重力并行 |

### 安全与运维要点

- 串口只由 **一个** agent 占用；launch 默认 `start_agent:=false`
- deactivate 前：`ros2 service call /microros_stm32h723_hardware/safe_stop std_srvs/srv/Trigger {}`
- 单电机冒烟：固件 mask `0x01` + `single_motor_smoke_test.py`（不要直接上七关节插件）
- 纯重力调参：`--position-controller=none --gravity=true`
- Joint4 标定：`--mode=calibration`；服务 `begin_pose` / `capture_pose` / `release_pose` / `fit` / `reset`
- MIT 增益文件：`arm_mit_gains.yaml`（RETURN/HOLD/TRACK/IDLE）；重力专用 `gravity_mit_gains.yaml`（kp=0）

包内说明：`src/plugin/microros_stm32h723_bridge/README.md`。

## MoveIt 如何接到后端

`rebotarm_moveit_config` 通过 `moveit_controllers.yaml` 把规划结果交给名为 `arm_controller` 的 `FollowJointTrajectory` 目标。

- **demo**：mock hardware（`mock_components/GenericSystem`）
- **gazebo_demo**：Gazebo 的 `arm_controller`
- **真机 micro-ROS**：需使用轨迹控制器命名一致的 profile（arm launch 上的 `arm_trajectory_controller`，或自行对齐 MoveIt 配置）

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
