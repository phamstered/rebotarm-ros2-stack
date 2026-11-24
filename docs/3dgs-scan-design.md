# 3D Gaussian Splatting 采集契约与环绕扫描（设计草案）

> 状态：**草案 / 尚未实现**。本文只冻结 v1 契约，供人工审阅。本仓库没有采集节点、环绕轨迹节点、launch、测试或服务；按本文不能启动扫描，也不能把 splat 接进控制回路。
>
> 已核对的现网基线（2026-11-24 `main`）：左腕相机 launch、`camera_mount_joint` 手眼、`/robot_feedback/tf_joint_states`、数据采集门禁、`arm_trajectory_controller` 与六轴 2 rad/s、2 rad/s² 规划上限。与这些源文件冲突时，以源文件为准。

::: warning 尚未实现
现有腕部相机、MoveIt 与录制门禁的行为仍以代码和 [数据采集质量](./data-collection)、[构建与运行](./build-run)、[控制栈与后端](./controllers) 为准。下文的数据集、环绕路径和 splat 导入都还没有落地。
:::

本仓库负责两件事的规范性约定：**采集契约**（怎样把一帧彩色图变成 Nerfstudio 风格的相机位姿）和 **环绕轨迹**（怎样在现有 Jetson MoveIt / Trajectory 路径上停稳再拍）。Desk 负责会话编排、路径确认和 Mac 上的 Brush；固件本切片不改。这两侧只复述已经约定的边界，不新增 UI 或固件协议。

## 1. 目标与非目标

### 目标

- 离线建立一份场景 splat。采集在 Jetson 上完成，重建在数据集拷到 Mac 之后进行。
- 数据集是 Nerfstudio 风格的 `transforms.json` 加 RGB 文件。对齐深度可以附带，Brush 不依赖它。
- 每帧位姿是图像 stamp 上的光学帧，写到 ROS `base_link`（Z 朝上、单位米）。导出的 `.ply` 仍在这个坐标系里。
- 环绕物体拍摄：半球或 1–2 圈、光轴指向物体中心、MoveIt 位姿目标、航点之间关节空间 PTP、停稳再拍。
- 航点运动和静置期间，已有 Trajectory / JTC 保持在 MIT `TRACK`，命令流不中断。

### 非目标（v1）

- 不在 Jetson 上做在线 / 实时 3DGS，也不把 splat 用于碰撞、规划、抓取或实时控制。
- 不替换 Desk Robot View 已有的单帧 RGBD 预览（RCD1）。splat 是叠加上去的离线模型。
- Mac 与 Desk 不加入 DDS，不发布 `/stm32/joint_commands`。
- 不改固件，不为本路径切换 POS_VEL。默认固件控制仍是 MIT。
- 不把 OpenSplat 打进任何交付物（AGPL）。不用 CUDA-only 的 nerfstudio / gsplat splatfacto 作为 Mac 训练器。
- 不做连续笛卡尔 LIN，也不在航点静置时拆掉控制栈。

## 2. 已约定的系统边界

| 角色 | 谁来做 | v1 边界 |
|------|--------|---------|
| 采集与运动 | Jetson 上的现有 ROS 2 栈 | 相机、TF、门禁、环绕规划与 `TRACK` 执行都在这里 |
| 会话编排 | Desk | 开始 / 停止采集会话；预览路径；操作者确认后才允许执行；沿现有 SSH / 会话把数据集拷到 Mac |
| 重建 | Mac 上的 Brush **CLI sidecar** | 不编译进 Tauri / WebView。关闭自动重定向和 up-axis 变更 |
| 查看 | Desk Robot View | 优先 Spark，叠在现有 Three.js 场景上，与 URDF 和实时点云共用 `base_link`。可独立开关。不由 TCP 拖拽预览驱动。Spark 若无法共享当前渲染循环，v1 回退 GaussianSplats3D |
| 固件 | 现网 MIT | 不改协议。`TRACK` 命令间隙 >100 ms 落入 `HOLD`；>500 ms 锁存 fault 11 |

3DGS 是离线场景模型。它不参与实时控制，也不是 RCD1 那路单帧 RGBD 预览的替代品。

物体中心由 Desk 的视觉 Pick，或 Desk 已有的桌面 AABB 加上 keep-out 半径给出。本仓库不定义 Pick UI，也不新造一个桌面检测话题。Jetson 侧只消费这次会话已经给出的中心、keep-out，以及调用方写入 Planning Scene 的障碍。

## 3. 规范性采集契约

### 3.1 相机入口

采集使用已经在跑的左腕相机，不另起一套驱动：

```bash
ros2 launch rebotarm_gazebo left_wrist_camera.launch.py
```

`left_wrist_camera.launch.py` 固定：

| launch 参数 | 值 |
|-------------|----|
| `camera_namespace` | `cameras/left_arm` |
| `camera_name` | `wrist` |
| `tf_prefix` | `left_arm/` |
| `enable_color` / `enable_depth` | `true` |
| `align_depth.enable` | `true` |
| `enable_sync` | `true` |

该 launch 只包含 RealSense `rs_launch.py`，并发布 `camera_link` → `left_arm/wrist_link` 的零位姿静态 TF。它不启动 `robot_state_publisher`。关节 TF 来自已经运行的真机臂栈。

仓库文档今天写的是话题前缀 `/cameras/left_arm/wrist/`（见 [构建与运行](./build-run)）。完整话题名由 `realsense2_camera` 按上表参数发布，源码里没有把这三串再硬编码一遍。v1 采集订阅这三路：

| 用途 | 话题 |
|------|------|
| 彩色 | `/cameras/left_arm/wrist/color/image_raw` |
| 对齐深度（可选） | `/cameras/left_arm/wrist/aligned_depth_to_color/image_raw` |
| 彩色内参与光学帧 | `/cameras/left_arm/wrist/color/camera_info` |

多相机时沿用现有 `serial_no:=...`。序列号写入第 3.6 节的元数据。

### 3.2 坐标系与手眼

```text
base_link
  └─ 关节链（动态 TF：20 Hz /robot_feedback/tf_joint_states，经 stamp 准入）
       └─ link6
            └─ camera_link                         camera_mount_joint，2026-09-12 手眼
                 └─ left_arm/wrist_link            静态单位变换
                      └─ <color optical frame>     以 camera_info.header.frame_id 为准
```

`link6` → `camera_link` 以 URDF `camera_mount_joint` 为准，标定四元数而不是导出里被注释掉的那组 Euler：

| 项 | 值 |
|----|----|
| 父 / 子 | `link6` → `camera_link` |
| 四元数 xyzw | `[0.598366, 0.00397963, 0.801167, 0.00856162]` |
| 平移（米） | `[0.0520795, -0.0139823, 0.0672182]` |
| URDF `rpy` | `0.058487598396866 -1.282446543624948 3.076557194400623`（由归一化四元数转换而来） |

`camera_link` → `left_arm/wrist_link` 是单位静态 TF。光学帧挂在 RealSense 驱动发布的后续帧上。本仓库没有把光学帧名字写死。在当前 `tf_prefix` 与 `camera_name` 下，驱动的惯例名是 `left_arm/wrist_color_optical_frame`；查找时仍以 `camera_info.header.frame_id` 为准。

动态 TF 只来自 20 Hz、做过时间戳准入的 `/robot_feedback/tf_joint_states`。`robot_state_publisher` 订阅的是这一路。ros2_control 的 `/joint_states` 在硬件未配置时可能停在旧姿态，不能用来给相机配位姿。准入规则见 [数据采集质量](./data-collection)。

### 3.3 什么样的帧可以留下

开录前仍跑现有硬门禁：

```bash
./scripts/check_collection_ready.sh
```

脚本要求 `/stm32/joint_states` 与 `/stm32/status` 的 `header.stamp` 非零，且 `/stm32/status` 满足 `mode==TRACK(1)`、`fault_code==0`。这只是会话能否开始的预检。数据集还要对每一帧做丢弃。

一帧彩色图进入数据集，必须同时满足：

1. 图像 `header.stamp` 非零。
2. 与之配对的 `color/camera_info` 同一 stamp，内参和 `frame_id` 都来自这一条消息。
3. 图像 stamp 当时的 `/stm32/status` 仍是 `mode==1` 且 `fault_code==0`。`HOLD`（`mode==0`）或任何非零 fault 都丢弃，包括 fault 11（`COMMAND_LOSS_LATCHED`）。
4. `last_command_sequence` 没有在该帧附近停住。序号停滞沿用现有采集清单，表示命令流已经卡住。
5. `tf2` 能在 **图像 stamp** 上查到第 3.4 节的变换。查不到就丢，包括缓冲区里没有这个时刻、外推到尚未准入的反馈，或光学帧名字尚未出现。

用 `tf2` 在相邻 20 Hz 样本之间插值是允许的。用最新关节角、`Time(0)` 或“现在”的 TF 去配一张过去的图像是不允许的。对齐深度缺失不丢弃彩色关键帧；若保存深度，只保存与该彩色帧同一 stamp 的对齐深度。

### 3.4 位姿：光学帧，相机到世界

每一帧的位姿来源只有一次查找：

```text
T_base_optical = lookup_transform(
    target_frame = "base_link",
    source_frame = camera_info.header.frame_id,
    time         = image.header.stamp)
```

ROS 2 的 `lookup_transform(target, source, time)` 把 **source 系中的点** 变到 **target 系**。因此 `T_base_optical` 就是光学帧相对 `base_link` 的位姿，也就是相机到世界 `c2w_optical`。不要取逆，除非实现用的是相反的 target/source 顺序。

`camera_link` 和 `left_arm/wrist_link` 是相机机体帧，不是光学帧。它们可以写进元数据供对账，不能拿来填 `transform_matrix`。

世界系是 ROS `base_link`：Z 朝上，单位米。不把世界改成 Y-up，也不做任何自动摆正。

### 3.5 光学轴到 Nerfstudio / OpenGL

ROS 光学帧：X 向右，Y 向下，Z 向前（视线沿 +Z）。

Nerfstudio / OpenGL 相机：X 向右，Y 向上，Z 向后（视线沿 −Z）。

相机原点不变。只把相机自身的 Y、Z 轴反向。列向量、与 NumPy `@` 相同的约定下，右乘对角阵：

```text
c2w_ns = c2w_optical @ diag(1, -1, -1, 1)
```

右乘作用在相机轴上。记 `c2w_optical = [R | t]`，则

```text
c2w_ns = [ R @ diag(1, -1, -1) | t ]
```

第四列 `t` 原样保留，所以平移不变。新的三列是 OpenGL 相机的 +X、+Y、+Z 在 `base_link` 里的方向：

- OpenGL +X = 光学 +X（图像仍向右）
- OpenGL +Y = −光学 Y（图像向上；光学 Y 向下）
- OpenGL +Z = −光学 Z（OpenGL 的视线是 −Z，与光学 +Z 同一条射线）

数值例子（列向量）。光学轴取成：+X = 世界 +X，+Y = 世界 −Z，+Z = 世界 +Y，原点 `(0.30, -0.10, 0.40)` 米。

```text
c2w_optical =                 c2w_ns = c2w_optical @ diag(1,-1,-1,1)
[[1,  0, 0,  0.30],           [[1, 0,  0,  0.30],
 [0,  0, 1, -0.10],            [0, 0, -1, -0.10],
 [0, -1, 0,  0.40],            [0, 1,  0,  0.40],
 [0,  0, 0,  1.00]]            [0, 0,  0,  1.00]]
```

原点仍是 `(0.30, -0.10, 0.40)`。OpenGL −Z 列取反后是 `(0, 1, 0)`，等于光学视线。OpenGL +Y 是 `(0, 0, 1)`，等于世界向上。

左乘会改世界轴，而不是相机轴。同一例子：

```text
diag(1,-1,-1,1) @ c2w_optical =
[[1, 0,  0,  0.30],
 [0, 0, -1,  0.10],
 [0, 1,  0, -0.40],
 [0, 0,  0,  1.00]]
```

原点从 `(0.30, -0.10, 0.40)` 变成 `(0.30, 0.10, -0.40)`。这就是把场景翻成另一套 up 轴。v1 禁止这条路径，也禁止 Brush 或导入步骤再做一次等价的自动摆正。

`transform_matrix` 在 JSON 里按行写出，只是列向量矩阵的存储顺序。点仍是列向量，`diag` 仍在右边。不要把“按行存储”理解成行向量约定，也不要因此改成左乘。使用行向量的库先换到这一约定，再套上面的公式。

### 3.6 数据集

v1 目录：

```text
<session_dir>/
  transforms.json
  capture_metadata.json
  images/
    000001.png
  depth/                 # 可选；Brush 不读
    000001.png
```

`transforms.json` 是 Nerfstudio 风格。世界系已经是 `base_link`，文件里不写会改变 up 轴的 `applied_transform`。每个留下的关键帧：

| 字段 | 来源 |
|------|------|
| `file_path` | 该帧 RGB 文件 |
| `transform_matrix` | 4×4 列向量矩阵，JSON 按行写出，即 `c2w_ns` |
| `fl_x` / `fl_y` / `cx` / `cy` | 该帧 `sensor_msgs/CameraInfo.k` |
| `w` / `h` | 该帧 `width` / `height` |
| `camera_model` | `OPENCV` |

内参只读该帧 `CameraInfo`，采集侧不去畸变，也不改写成 `PINHOLE`。`distortion_model` 为 `plumb_bob` 且 `d` 不是全零时，按 OpenCV 顺序把 `k1`、`k2`、`p1`、`p2` 以及存在的 `k3` 写入对应帧。不同帧的 `k` 不一致时按帧保留，不取平均。图像编码（PNG / JPEG）不锁定，`file_path` 必须指向实际文件。

关键帧，而不是每张相机图都入库：

- 选择器按位姿变化抽帧，密度约 **2–5 Hz** 量级。相机自身可以更高帧率出图。
- v1 运动是停稳再拍。航点之间的运动帧不入库。一个通过门禁的静置只留一帧；静置期间位姿几乎不变，不要连写多帧去凑 2–5 Hz。
- 2–5 Hz 是连续运动时的抽帧密度，不是静止点上的产品指标，也不是固定视点个数。平移 / 旋转阈值是实现旋钮。

`capture_metadata.json` 至少包含这些对账字段：

| 字段 | 内容 |
|------|------|
| `schema` | `rebotarm.3dgs_capture.v1` |
| `units` | 长度米，角度弧度 |
| `world_frame` / `world_up` | `base_link` / `+Z` |
| `c2w_convention` | `camera_to_world`，并且已经右乘 `diag(1,-1,-1,1)` |
| `applied_world_transform` | `null` |
| `color_topic` / `camera_info_topic` | 第 3.1 节的两路 |
| `depth_topic` / `depth_required` | 对齐深度话题 / `false` |
| `optical_frame_source` | `camera_info.header.frame_id`；并记录本会话实际出现的帧名 |
| `body_frames_not_used_for_pose` | `camera_link`、`left_arm/wrist_link` |
| `hand_eye` | 第 3.2 节的关节名、`2026-09-12`、xyzw、平移 |
| `static_mount` | `camera_link` → `left_arm/wrist_link` 单位变换 |
| `tf_joint_source` | `/robot_feedback/tf_joint_states` |
| `camera_serial` | launch `serial_no` 或设备序列号；未知则空字符串并标明未知 |
| `distortion_model` | 来自 `CameraInfo` 的原值 |

丢帧原因与第 4 节的规划跳过原因分开记在会话日志或 metadata 的计数里，供事后解释重影和错位。原因码用：`bad_stamp`、`camera_info_mismatch`、`hold_or_fault`、`sequence_stall`、`tf_unavailable`。

### 3.7 训练导出与回到 Robot View

Brush 在 Mac 上以 CLI sidecar 运行。训练和导出必须关闭自动重定向 / up-axis 变更。导出的 `.ply` 保持在 `base_link`。Desk 导入时不再做轴交换，也不按 TCP 拖拽预览去摆这个模型。

具体 CLI 开关属于 Desk，本文不规定参数名。验收标准是：导入后的 splat 与 URDF、实时点云共用同一个 `base_link`，相机原点与采集时的 `t` 一致。

## 4. 规范性环绕轨迹

### 4.1 视点

输入（本会话已由 Desk 给出，本文不定义其 UI）：

- 物体中心，`base_link` 下的米制位置
- keep-out 半径，或调用方给出的等价 AABB
- 桌面和其他障碍，由调用方写入现有 Planning Scene。`real_move_group.launch.py` 今天不会自动生成桌面碰撞体

栈在中心周围生成半球，或桌面以上的 1–2 个纬线圈。每个视点：

- 相机原点在该球或圈上，光学帧 +Z 指向中心
- 光学帧姿态取唯一解：`z = normalize(center - origin)`，`x = normalize(cross(z, world_up))`，`y = cross(z, x)`，`world_up = (0,0,1)`。这样图像上方大致朝向世界 +Z
- 正上方视点与 `world_up` 平行，`cross` 退化。v1 跳过该视点，原因 `singular_view`，或者改用一个固定水平参考轴并在覆盖报告里写明。不要随机滚转

视点还要避开三件事，能用现有碰撞模型表达多少就做多少：

- 桌面与 Planning Scene 里的障碍
- 物体 keep-out
- 腕部相机被手臂挡住：光学原点到物体中心的线段与臂碰撞模型相交则跳过，原因 `self_occlusion`

不可达（`unreachable`）或碰撞（`collision`）的视点直接跳过，并计入覆盖报告。禁止放宽 IK、忽略碰撞或重复求解去硬凑一个坏姿态。

v1 的视点数量起点是大约 **16–24** 个，例如两圈各 8–12 个。这是起步建议，不是产品承诺的固定张数。观察距离和仰角随物体大小变化，本文不锁毫米级常数。

规划用的是 `link6`，采集用的是光学帧。生成视点时先得到光学帧位姿，再沿静态链 `link6` → 光学帧（`camera_mount_joint`、单位挂载、RealSense 光学外参）换算成 `link6` 在 `base_link` 下的位姿目标。不要把光学帧目标直接交给 `link6` IK。这条外参链不随关节变化，不要用某一帧最新关节角代替它。

### 4.2 执行

执行路径只使用已经存在的真机规划：

```bash
ros2 launch rebotarm_moveit_config real_move_group.launch.py
```

该 launch 只起 `move_group`，不创建硬件，也不切换控制器。扫描前 `arm_trajectory_controller` 必须已经 active。规划组是 `arm`（joint1–joint6；SRDF 里 `end_effector` 的 `parent_link` 是 `link6`）。位姿目标之间用关节空间 PTP。v1 不用连续 LIN / 笛卡尔速度流。

真实六轴规划上限保持 **2 rad/s** 与 **2 rad/s²**（`rebotarm_moveit_config` 的 `joint_limits.yaml` 与 `real_move_group.launch.py`）。这是规划上限，不修改固件限幅、MIT 增益或重力补偿。夹爪不在 `arm` 组里；扫描不为夹爪单独下发轨迹。

MoveIt 把轨迹交给 `arm_trajectory_controller` 的 `follow_joint_trajectory`（`FollowJointTrajectory`）。控制器认领 position 与 velocity，硬件以 MIT `TRACK` 写出 `/stm32/joint_commands`。

每个航点是停稳再拍：

1. PTP 到换算后的 `link6` 目标。
2. 静置到关节速度接近 0，且第 3.3 节的门禁通过。
3. 写下一帧关键帧。
4. 再去下一个可达航点。

静置时长是实现参数，v1 不锁定秒数。静置的含义是“命令还在流、臂停在航点上”，不是“这段轨迹结束了，控制栈可以停”。

### 4.3 静置时命令流不能断

扫描期间保持进入扫描前的控制栈：

- `arm_trajectory_controller` 保持 active，不 deactivate，不切回 teleop / VLA，不重启 ControllerManager
- 不调用 `safe_stop`，不拆硬件生命周期
- 已 active 的重力控制器保持原样（它只占 effort，与轨迹的 position / velocity 并存）
- 不申请 Motor Control V2 租约，不把这次运动改成 POS_VEL

航点运动和静置都留在 MIT `TRACK`。JTC 必须按控制周期继续写出命令。一段 `FollowJointTrajectory` 若在静置开始时已经执行完，实现要让该控制器继续写出最后一帧：位置保持在航点，速度约为 0，直到这一帧采集结束并开始下一段 PTP。

可观察的判据：整段扫描（含静置）期间 `/stm32/status` 保持 `mode==TRACK(1)`、`fault_code==0`，且 `last_command_sequence` 持续递增。

固件侧已有约定，本切片不改：`TRACK` 命令间隙 **>100 ms** 落入 `HOLD` 且不置故障；**>500 ms** 锁存 **fault 11**（已使能时保持 HOLD，否则 DISABLE）。本仓库 [一键重力 / 参数辨识](./gravity-identification) 已按 executor 现网记下 `COMMAND_TIMEOUT_MS=100` 与 `COMMAND_LOSS_LATCH_MS=500`。静置若停发命令，会直接踩中这两条，随后第 3.3 节必须丢帧，臂还可能下垂。

这两个数不是 ros2_control 的反馈超时。真机 arm xacro `rebotarm.arm.micro_ros.xacro` 的 `state_timeout_ms` 是 **500**，表示状态反馈过期。`config/micro_ros_system.yaml` 与 `rebotarm.micro_ros.xacro` 里的 `state_timeout_ms: 100` 是另一份硬件默认值，同样不是固件命令间隙。实现时不要把三者合成一个参数。

### 4.4 预览、确认与覆盖报告

生成路径不等于执行。Desk 在 Robot View 里预览这条路径，操作者确认之后 Jetson 才开始运动。确认控件属于 Desk。确认之前，扫描逻辑不向 `/stm32/joint_commands` 发布命令，也不为了预览去切换控制器。

预览至少包含：有序的光学帧原点与光轴（`base_link`）、对应的 `link6` 目标、被跳过的视点及原因。传输形式（沿现有 SSH / 会话传递的参数或文件）尚未冻结；冻结的是“未确认不执行”，以及 Mac / Desk 不加入 DDS。

覆盖报告与采集丢帧分开：

| 原因码 | 含义 |
|--------|------|
| `unreachable` | IK 无解 |
| `collision` | 与桌面、场景或 keep-out 碰撞 |
| `self_occlusion` | 视线段与臂碰撞模型相交 |
| `singular_view` | 正上方等光轴退化 |
| `skipped_unconfirmed` | 路径没有得到操作者确认，未执行 |

报告给出请求视点数、执行视点数和跳过列表。跳过是正常结果，不是要重试到全部成功。

## 5. Desk 与固件摘要

以下各项已经约定，细节不在本仓库展开：

- Desk 编排开始 / 停止采集、路径预览与确认、把数据集拷到 Mac、启动 Brush CLI、再把 `.ply` 导入 Robot View。
- 查看器优先 Spark，叠在现有 Three.js 上，坐标系与 URDF / 实时点云相同，可独立关闭。共享不了当前渲染循环时，v1 使用 GaussianSplats3D。OpenSplat 不打包。
- RCD1 单帧 RGBD 预览保持原样。
- 固件保持现网 MIT。本路径不使用 POS_VEL，不增加模式位或消息。

## 6. 审阅时核对的不变量

实现 PR 以后另行提交。审阅本文时只要看契约是否会被实现成另一种坐标系或另一条控制路径：

1. `transform_matrix` 是图像 stamp 上的光学帧 `c2w`，并且右乘了 `diag(1,-1,-1,1)`。原点不变，世界仍是 `base_link` Z-up。
2. 数据集不使用 `camera_link`、最新 `/joint_states`，也不使用会改 up 轴的自动摆正。
3. 环绕是 MoveIt `link6` 位姿目标加 PTP 和停稳再拍。不可达或碰撞的视点被跳过并上报。
4. 静置期间 JTC 继续流 `TRACK`。命令间隙仍按现网 100 ms / 500 ms 处理。
5. Mac 与 Desk 不进入命令路径。splat 不参与碰撞、规划或实时控制，也不替换 RCD1。
