# 控制路径（CAN vs micro-ROS）

从臂有两条硬件路径；仿真路径走 Gazebo + ros2_control。选路径时不要混用同一串口/同一组 effort 接口上的冲突控制器。

## 路径总览

| 路径 | 入口 | 命令去向 | 典型 launch / 脚本 |
|------|------|----------|-------------------|
| **Gazebo** | `relay` `mode: gazebo` | `FollowJointTrajectory` → `arm_controller` | `relay_to_gazebo.launch.py` + `start_gazebo.sh` |
| **CAN 真机** | `relay` `mode: real` | `/joint_commands` → `rebotarm_hw_driver` | `relay_to_real.launch.py` + `start_jetson.sh --mode=real` |
| **micro-ROS** | `relay` `mode: ros2_control` 或 teleop | `arm_teleop_controller` / trajectory → `microros_stm32h723_bridge` | `relay_to_microros_arm.launch.py`、`start_arm_teleop.sh` |

配置文件：

- Gazebo：`src/joint_state_relay/config/relay.yaml`
- CAN real：`src/joint_state_relay/config/relay_real.yaml`
- micro-ROS / ros2_control：`src/joint_state_relay/config/relay_microros_arm.yaml`

## Gazebo 路径

1. Jetson：`start_jetson.sh --mode=gazebo`（主臂 + relay，domain 0）
2. Ubuntu：`start_gazebo.sh`（仿真 + controller_manager + domain bridge + relay）
3. relay 通过 trajectory action/topic 驱动仿真 `arm_controller`

硬件插件为 Ignition：`ign_ros2_control/IgnitionSystem`。

## CAN 路径（`rebotarm_hw_driver`）

- Jetson 经 `can0` 与从臂通信
- relay 发布 `sensor_msgs/JointState` 到 `/joint_commands`
- 驱动侧做 MIT 阻抗控制与单位换算（含夹爪 m → 电机 rad）

夹爪校准参数在 `relay_real.yaml`（sign / offset）；驱动内部另有电机换算系数，改动后需实测。

## micro-ROS 路径（`microros_stm32h723_bridge`）

C++ `hardware_interface::SystemInterface`：

```
MoveIt / JointTrajectoryController / teleop
        │  ros2_control read()/write()
        ▼
microros_stm32h723_bridge (MIT + 可选重力前馈)
        │  DDS JointCommand / JointState
        ▼
micro-ros-agent (serial)
        │  XRCE-DDS
        ▼
STM32H723 固件（详见 rebotarm-executor，本仓库不展开固件）
```

要点：

- 串口只应由 **一个** agent 占用；launch 默认 `start_agent:=false`，真实部署常用 systemd 管理的 agent
- `gravity_compensation_controller` 只声明 effort 命令接口；勿与 `joint4_calibration_controller` 同时激活（争用同一 effort 接口）
- 安全停机：deactivate 前调用 `/microros_stm32h723_hardware/safe_stop`
- 单电机冒烟：固件 `active_joint_mask=0x01` + `single_motor_smoke_test.py`（不要直接上七关节插件）

遥操 / 标定入口示例：

```bash
./scripts/start_arm_teleop.sh \
  --position-controller=none --gravity=true --clean=true

./scripts/start_arm_teleop.sh --mode=calibration --clean=true
```

插件 README：`src/plugin/microros_stm32h723_bridge/README.md`。

## MoveIt

`rebotarm_moveit_config` 通过 `moveit_controllers.yaml` 将规划结果交给 `arm_controller`（`FollowJointTrajectory`）。controller 与 hardware 的接口名匹配机制见 [ros2_control 关联机制](./ros2_control_association)。

## 选择建议

- 开发仿真与轨迹逻辑 → **Gazebo**
- 现有 CAN 从臂、不经 ros2_control 插件 → **CAN + rebotarm_hw_driver**
- 统一 ros2_control / MoveIt / 重力补偿 / Joint4 标定 → **micro-ROS 桥**
