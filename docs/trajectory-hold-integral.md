# 可选轨迹积分补偿（实验）

当前点位起点检查要求 JTC 保持参考与真实反馈偏差不大于 0.005 rad，并连续稳定 200 ms。本功能不改变这些门禁，也不改变 MoveIt 当前起点、碰撞检查、关节限位或固件保护。

## 现场诊断

2026-10-31 在当前姿态只读采集 15 秒，511 个稳定参考样本中，目标速度为零，硬件始终 TRACK、fault=0，无写入丢样。误差在整个保持窗口内恒定：

|关节|参考−实际 rad|角度 °|现有 Kp|对应 P 项 Nm|
|---|---:|---:|---:|---:|
|joint1|+0.005574|+0.319|40|+0.223|
|joint2|+0.021195|+1.214|40|+0.848|
|joint3|−0.046378|−2.657|40|−1.855|
|joint4|−0.002809|−0.161|15|−0.042|
|joint5|+0.001742|+0.100|15|+0.026|
|joint6|0|0|15|0|

界面只报告第一个超限关节时，joint1 的轻微超差会遮住 joint2/3 更大的偏差。现场已确认无机械阻挡且零位已校准。历史采集中 joint2/3 接近目标的方向变化后，保持误差出现反号。这支持静摩擦/方向相关误差参与的判断，不能仅从单姿态拟合一个通用重力缩放或常量偏置，也不能据此排除模型/负载误差。

## 实现边界

- `MicroRosSystem` 可通过独立 YAML 读取 `ki`、`max_effort`、`slew_rate`，均为 7 个协议槽位。未指定文件时完全关闭；夹爪槽位禁止启用。原 MIT 增益文件、RETURN、TRACK、HOLD 参数均不改写。
- 只在 TRACK 指令、同一关节的 position/velocity 同时被认领、且新鲜 HardwareStatus 确认 TRACK、无故障、所需关节 active 时积分。积分施加到 `effort` 前馈字段，不改写参考位置、JointState 或速度。
- 每周期先按 Ki 积分，再限制补偿幅度与变化速率。最终 `effort` 字段仍限制在原 `effort_limits` 内；向饱和方向停止累积，允许反向消退。无效/大于 50 ms 的周期不补算积分。
- 非有限配置或输入被拒绝；任何关节使整帧失败时不提交部分积分状态。激活、控制器释放/重新认领、非 TRACK 模式和 DISABLE 清除对应状态。DISABLE 仍立即清零全部控制字段并使用新 sequence。
- 下位机反馈非 TRACK、故障、所需关节不 active、序列回退或状态过期时清理补偿；即使 DISABLE→TRACK 在两次控制周期之间发生，也不继承旧积分。此逻辑只管理补偿状态，不替代已有模式重启/接管流程。
- 该前馈限幅不是电机总转矩限幅：电机内部 MIT PD 项仍参与总输出。积分状态清理也不等于跨模式无冲击切换保证，需单独验证启停与 RETURN。

这是用于消除静态负载残差的可选控制扩展，不是已校准的摩擦模型，不保证消除所有负载下的低速走停。持续外力、受阻或模型错误可能导致补偿饱和；此时起点检查仍会拒绝点位。不能通过提高上限掩盖现场异常。

## 审核与现场验证入口

示例文件 `config/trajectory_integral.example.yaml` 仅启用前三轴，其他协议槽位为零。系数根据当前保持窗口提供实验起点，尚未真机标定。默认启动脚本、桌面一键模式和普通 launch 均不自动启用。

必须先完成现场安全确认，并通过既有安全停机入口确认 DISABLE、清理旧控制会话；不能向仍运行的 ControllerManager 重复启动第二套硬件。人工审核后，可在明确清空的测试环境中选择性启动：

```bash
ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py \
  position_controller:=trajectory enable_gravity_controller:=true \
  trajectory_integral_config:="$(ros2 pkg prefix microros_stm32h723_bridge)/share/microros_stm32h723_bridge/config/trajectory_integral.example.yaml"
```

验证顺序为隔离测试、当前姿态的受监督保持，再验证小幅运动和停机。首轮可用 YAML 将未验证关节的三个参数均设为零，逐轴验证；记录参考/实际、速度、effort、饱和、模式和故障。姿态变化后需要重新检查，不从本次静态数据推断全部工作空间安全。

本次只做隔离测试，未把实验配置写入生产控制栈，也未启用真机积分。

## 隔离验证记录

2026-10-31，Jetson ROS 2 Humble，独立工作区 `~/.cache/robot-console-validation/velocity-36`，`ROS_DOMAIN_ID=230`。未启动串口驱动或连接真实 MCU：

```bash
colcon build --symlink-install --packages-up-to microros_stm32h723_bridge \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select microros_stm32h723_bridge --event-handlers console_direct+
colcon test-result --verbose
python3 src/plugin/microros_stm32h723_bridge/test/smoke_trajectory_velocity.py
python3 src/plugin/microros_stm32h723_bridge/test/smoke_trajectory_velocity.py --integral
```

包内测试覆盖下位机重置、故障、反馈过期、限幅、配置拒绝、接口切换及 DISABLE；另有 relay pytest。两次实际 JTC → 硬件插件 → 模拟 MCU 链路验证分别覆盖默认关闭和显式启用，检查补偿字段、上限、Teleop 切换清理、重新激活及 DISABLE 全字段清零。模拟 MCU 仅提供位置跟随滞后，不模拟真实摩擦、弹性或重力，因此该结果不能替代真机收敛与启停验证。
