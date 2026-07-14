# ros2_control Controller 与 HardwareComponent 关联机制

## 核心原理

`ros2_control` 中，controller 和 hardware component **没有显式绑定关系**。它们通过**接口名称自动匹配**：

```
HardwareComponent 导出接口          Controller 声明需求接口
    joint1/position (cmd)  ◄─────  joint1/position (command_interface)
    joint1/position (state) ─────► joint1/position (state_interface)
    joint1/velocity (state) ─────► joint1/velocity (state_interface)
```

`controller_manager` 负责在运行时完成匹配，只要 **关节名 + 接口类型名** 一致即可。

## 配置层级

### 1. URDF/xacro — 声明硬件提供的接口

文件：`rebotarm_moveit_config/config/rebotarm.ros2_control.xacro`

```xml
<ros2_control name="RebotArmSystem" type="system">
  <hardware>
    <!-- 插件全限定名，controller_manager 据此加载 .so 并实例化 -->
    <plugin>rebotarm_hw_interface/RebotArmSystem</plugin>
    <!-- 可选：传递参数给硬件插件 -->
    <param name="config_path">/path/to/arm.yaml</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-2.8</param>
      <param name="max">2.8</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- joint2 ~ joint6 同理 -->
</ros2_control>
```

**关键点：**
- `<plugin>` 是 C++ 插件的 `PLUGINLIB_EXPORT_CLASS` 注册名
- `<command_interface>` = 硬件可接受的写入通道
- `<state_interface>` = 硬件可提供的读取通道
- 每个 `<joint>` 的 name 必须与 URDF 中的 joint name 一致

### 2. ros2_controllers.yaml — 声明 Controller 需要的接口

文件：`rebotarm_moveit_config/config/ros2_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position          # 匹配 xacro 中的 <command_interface name="position">
    state_interfaces:
      - position          # 匹配 xacro 中的 <state_interface name="position">
      - velocity          # 匹配 xacro 中的 <state_interface name="velocity">
```

**关键点：**
- `joints` 列表中的名称必须与 xacro 中 `<joint name="...">` 一致
- `command_interfaces` / `state_interfaces` 中的名称必须与 xacro 中声明的接口名一致
- 不需要指定使用哪个硬件组件 — controller_manager 自动匹配

### 3. moveit_controllers.yaml — 告诉 MoveIt 如何发送轨迹

文件：`rebotarm_moveit_config/config/moveit_controllers.yaml`

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller

  arm_controller:
    type: FollowJointTrajectory
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    action_ns: follow_joint_trajectory
```

这一层只负责将规划结果通过 Action 接口发送给 `arm_controller`，不涉及硬件细节。

## 完整数据流

```
MoveIt move_group
  │  /arm_controller/follow_joint_trajectory (action)
  ▼
arm_controller (JointTrajectoryController)
  │  插值轨迹点，按 update_rate 周期输出
  │  write: command_interface["joint1/position"] = 目标位置
  │  read:  state_interface["joint1/position"]  ← 当前位置
  │         state_interface["joint1/velocity"]  ← 当前速度
  ▼
controller_manager (ros2_control_node)
  │  在 read() 和 write() 之间自动调度
  ▼
RebotArmSystem (你的 HardwareComponent)
  │  read()  → motorbridge.get_state() → 填充 state_interface
  │  write() → motorbridge.mit(读取 command_interface 中的目标值)
  ▼
CAN bus → 达妙电机
```

## Launch 文件配置

需要使用 `ros2_control_node` 加载硬件组件：

```python
control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        robot_description,        # 包含 xacro 中的 <ros2_control> 硬件声明
        ros2_controllers_yaml,    # controller 配置
    ],
    output="screen",
)

# 加载并激活控制器
spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["arm_controller", "joint_state_broadcaster"],
    output="screen",
)
```

## HardwareComponent 生命周期与硬件状态映射

```
on_init()       解析 URDF 参数，创建 motorbridge 实例
    │
on_configure()  arm.connect() — 建立 CAN 连接
    │
on_activate()   arm.enable() + arm.mode_mit(kp=0, kd=1.0) — 使能电机
    │             此后 read()/write() 按 update_rate 循环调用
    │
on_deactivate() 平滑回零 → arm.disable() — 安全停机
    │
on_cleanup()    arm.disconnect() — 断开连接
```

## 常见问题

| 问题 | 原因 | 排查 |
|------|------|------|
| controller 加载后无法收到指令 | xacro 中关节名拼写错误 | `ros2 control list_hardware_interfaces` 查看已注册接口 |
| 硬件插件未加载 | `<plugin>` 名称与 PLUGINLIB 注册名不一致 | 检查 `plugin.xml` 和 CMakeLists.txt |
| controller 一直处于 inactive | 未手动 activate | `ros2 control set_controller_state arm_controller active` |
| 接口数量不匹配 | xacro 中漏声明了某个关节的接口 | 对比 controller 的 joints 列表与 xacro 的 joint 列表 |
