# 一键全关节重力 / 参数辨识（设计草案）

> 房间共识摘要：编排在本仓库（ROS）；固件（rebotarm-executor）只做 **meas + HOLD**；UI（robot-debug-desk）只调 action。  
> 状态：设计草案（未实现）。现有 `GravityCompensationController` + `Joint4CalibrationController` 仍保留；本设计是其 **一键、全关节、可脚本化** 的上位编排。

## 1. Goal / Non-goals

### Goal

- 提供 **一次 action 调用** 完成选中关节的重力缩放 / 力矩偏置辨识（可推广到全臂，不限于 joint4）。
- 动作名（服务器名）：**`/StartGravityID`**；类型名：`microros_stm32h723_bridge/action/GravityIdentification`。
- 输出可写入 YAML，并支持热加载进 `GravityCompensationController`。
- 安全：会话内固件保持 **HOLD**；abort / 超时 / 故障 → **立即 HOLD**；夹爪锁定；effort / 速度限幅。
- 明确 **固件接口契约**（模式、命令超时、fault code、反馈字段），便于 executor 更新 `docs/ros2-interface.md`。

### Non-goals

- 不在固件侧做辨识算法、姿态规划或 YAML 写入。
- 不替代日常重力前馈控制器；辨识会话与 `gravity_compensation_controller` / `joint4_calibration_controller` **互斥**（effort 接口争用）。
- 不做在线自适应控制、摩擦全模型、也不改 URDF 惯性参数（第一版只辨识 `gravity_scales` + `torque_biases`；payload 为可选子模式）。
- 不要求 Gazebo / CAN 路径第一版支持（目标后端：micro-ROS + STM32）。

---

## 2. Action 接口

### 2.1 命名

| 项 | 值 |
|----|-----|
| Action 类型 | `microros_stm32h723_bridge/action/GravityIdentification` |
| 服务器名 | `/StartGravityID` |
| 包内文件 | `action/GravityIdentification.action`（新建） |

### 2.2 Goal

```text
# GravityIdentification.action — Goal
uint32 joint_mask                 # bit0=joint1 … bit5=joint6；0 表示默认全臂 0x3F
uint8  mode                       # 0=FULL_ARM_GRAVITY，1=PAYLOAD_SUBMODE（旧 joint4 负载）
string[] pose_set_id              # 可选：命名姿态集；空则用内置激励集
float32 settle_time_s             # 到姿后静置时长，默认 1.5
float32 capture_window_s          # 采样窗，默认 1.0
bool    write_yaml                # true：写结果 YAML
bool    hot_reload                # true：辨识成功后热加载到 GravityCompensationController
string  output_yaml_path          # 空则默认 ~/.rebotarm/gravity_id_latest.yaml
float32 max_effort_nm             # 会话 effort 上限（叠加到每关节 limit 的额外帽）
float32 max_velocity_rad_s        # 激励运动速度上限，默认 0.15
float32 command_timeout_ms        # 期望固件命令超时（见 §4）；0=用固件默认
bool    lock_gripper              # 默认 true：会话内夹爪 HOLD 锁定
```

### 2.3 Feedback（ROS action，给 desk / 日志）

字段名固定，便于 UI 绑定：

```text
# Feedback
uint8  phase                      # 见 §4.1 PHASE_*
uint16 pose_index                 # 当前姿态序号（0-based）
uint16 pose_count                 # 总姿态数
uint32 active_joint_mask          # 当前参与关节
float32[] residual_nm             # 长度 6：当前窗内 mean(tau_meas - tau_model*scale) 或中间残差
float32[] measured_effort_nm      # 长度 6：窗内均值测得力矩
float32[] model_effort_nm         # 长度 6：Pinocchio 模型重力力矩
float32  rmse_running             # 已采姿态上的运行 RMSE（标量摘要）
uint16 fault_code                 # 透传 HardwareStatus.fault_code；0=无故障
string message                    # 人可读状态一行
```

### 2.4 Result

```text
# Result
bool     success
uint16   fault_code               # 结束时故障码；成功为 0
float32[] gravity_scales          # 长度 6；未参与关节保持原值或 NaN 约定见 YAML
float32[] torque_biases_nm        # 长度 6
float32[] rmse_per_joint          # 长度 6
float32  rmse_total
string   yaml_path                # 写出路径；未写则为空
string   message
```

### 2.5 固件侧需对齐的「会话」语义（给 executor）

ROS 编排节点在会话中只要求固件：

1. **持续接受并执行 `JointCommand`，模式为 `MODE_HOLD`（0）**，`position[]` = 当前保持位姿，`velocity[]` = 0，`effort[]` = 重力前馈（由 host 写入 effort 接口后由 `MicroRosSystem` 合成）。
2. **测量**：照常发布 `JointState`（position / velocity / effort）与 `HardwareStatus`。
3. **不**实现 `begin_pose` / `fit` 等服务；那些留在 ROS 或被 action 取代。

激励换姿时：host 可短暂切 `MODE_TRACK`（1）走到下一姿态，到位后立刻回到 `MODE_HOLD`。固件无需识别「辨识模式」枚举；**辨识会话 = 一串合法 HOLD/TRACK 命令 + 更严的超时/限幅约定**。

---

## 3. YAML schema + 热加载

### 3.1 Schema（`gravity_id_latest.yaml`）

```yaml
# reBotArm gravity identification output
schema_version: 1
identified_at: "2026-10-14T12:00:00+08:00"   # ISO-8601
source_action: StartGravityID
mode: full_arm_gravity                        # or payload_submode
joint_mask: 0x3F
joints: [joint1, joint2, joint3, joint4, joint5, joint6]

gravity_scales: [1.02, 0.98, 1.05, 1.10, 1.00, 1.00]
torque_biases_nm: [0.0, 0.05, -0.02, 0.12, 0.0, 0.0]
rmse_per_joint_nm: [0.08, 0.10, 0.09, 0.15, 0.05, 0.04]
rmse_total_nm: 0.11
pose_count: 12

# Optional payload sub-mode (legacy joint4 path)
payload:
  enabled: false
  frame: camera_link
  mass_kg: 0.0
  gravity_scale_joint4: 1.0
  bias_joint4_nm: 0.0

# Provenance
model_urdf_path: ""
notes: ""
```

### 3.2 热加载进 `GravityCompensationController`

第一版约定（实现时二选一，优先 A）：

- **A（推荐）**：控制器增加参数 `gravity_scales` / 未来 `torque_biases` 的动态更新（`rclcpp` parameter callback 或 `~/reload_gravity_params` `Trigger` + 读 YAML path 参数）。
- **B**：action 成功后 deactivate → 改参数 YAML → activate（打断前馈，仅调试可用）。

热加载后 effort 命令立即使用新 scale；**不**自动改 `arm_controllers.yaml` 持久默认值（持久化由用户/desk 另存）。

---

## 4. 状态机 + 安全（含固件 HOLD / 超时 / fault）

### 4.1 ROS 侧 phase（Feedback.phase）

| 值 | 名 | 含义 |
|----|----|------|
| 0 | `PHASE_IDLE` | 未运行 |
| 1 | `PHASE_ARMING` | 切控制器、夹爪锁、检查 HardwareStatus |
| 2 | `PHASE_ARMED` | 已进入会话，等待首个激励 |
| 3 | `PHASE_MOVING` | TRACK 前往姿态 |
| 4 | `PHASE_HOLD_SETTLE` | HOLD 静置 |
| 5 | `PHASE_CAPTURING` | HOLD 采样窗 |
| 6 | `PHASE_FITTING` | 最小二乘 / 回归 |
| 7 | `PHASE_WRITING` | 写 YAML / 热加载 |
| 8 | `PHASE_ABORTING` | 正在 abort → HOLD |
| 9 | `PHASE_DONE` | 正常结束 |
| 10 | `PHASE_FAILED` | 失败结束 |

### 4.2 会话状态机（摘要）

```text
IDLE → ARMING → ARMED → (MOVING → HOLD_SETTLE → CAPTURING)×N
     → FITTING → WRITING → DONE
任意阶段 abort / fault / 超时 → ABORTING → HOLD → FAILED
```

**Abort 契约（硬要求）**：

1. Action cancel 或内部故障 → 立即停止激励轨迹。
2. Host 对所有活动关节下发 **`JointCommand.mode = MODE_HOLD (0)`**，`position = 当前实测位置`，`velocity = 0`，`effort = 当前安全重力前馈（或 0，若前馈未就绪）`。
3. 夹爪保持锁定 HOLD。
4. 不自动 DISABLE；DISABLE 仅走现有 `safe_stop` 流程。
5. Feedback.phase = `PHASE_ABORTING`，然后 Result.success=false。

### 4.3 固件命令超时（command timeout）— executor 必写进 ros2-interface

| 字段 / 行为 | 约定 |
|-------------|------|
| 命令流 | Host 在会话内按现有桥接频率持续发 `JointCommand`（与正常 HOLD/TRACK 相同） |
| **`command_timeout_ms`** | 固件在 **最后一帧合法 JointCommand** 之后若超过此时长未收到新命令 → 判定命令丢失 |
| 默认建议 | 辨识会话：**100 ms**（与桥 `state_timeout_ms: 100` 同量级）；Goal 可覆盖，固件可拒绝过宽值 |
| 超时后固件行为 | 1) 锁存故障 2) **关节进入 HOLD（保持最后合法 position，effort=0 或固件安全值）** 3) `HardwareStatus.mode` 反映 HOLD 或安全态 4) `fault_code` 置命令丢失码 |
| Host 行为 | 读到该 fault → action ABORTING → 继续发 HOLD（若链路恢复）或引导 `safe_stop` |

> 固件 **不**需要单独的「GravityID 模式位」；超时策略可与现网命令丢失一致，但文档须写明：**辨识会话依赖同一 command-loss → HOLD 路径**，且 desk/编排不得在会话中停发命令。

### 4.4 Fault codes（`HardwareStatus.fault_code`，uint16）

与 executor `docs/ros2-interface.md` 对齐时建议采用下列 **命名与语义**（具体数值以 executor 现网表为准；若尚未占用，推荐值如下）：

| Code (建议) | 名 | 谁置位 | 会话内 ROS 动作 |
|-------------|-----|--------|-----------------|
| 0 | `FAULT_NONE` | — | 继续 |
| 0x0001 | `FAULT_CMD_LOSS` | 固件 | abort → HOLD |
| 0x0002 | `FAULT_STALE_SEQUENCE` | 固件 | abort；需按现网清序列/重启策略 |
| 0x0004 | `FAULT_MOTOR` | 固件 | abort → HOLD；禁止继续激励 |
| 0x0008 | `FAULT_EFFORT_LIMIT` | 固件或 ROS | abort → HOLD |
| 0x0010 | `FAULT_VELOCITY_LIMIT` | ROS（pose 安全）或固件 | abort → HOLD |
| 0x0020 | `FAULT_POSITION_DEVIATION` | ROS（HOLD 偏离参考过大） | 本姿态作废或整次 abort（可配置） |
| 0x0040 | `FAULT_GRIPPER_UNLOCKED` | ROS | abort（若要求 lock_gripper） |
| 0x0080 | `FAULT_SESSION_REJECTED` | 固件（可选：拒绝过宽 timeout） | 不进入 ARMED |

ROS Result/Feedback 的 `fault_code` **原样透传** `HardwareStatus.fault_code`；ROS 自检故障可用高位（例如 `0x8000 | local_code`）并在 `message` 说明，避免与固件码冲突。

### 4.5 固件反馈字段（meas）— 供采样

辨识采样只读现有话题（executor 保持字段稳定即可）：

| 来源 | 字段 | 用途 |
|------|------|------|
| `sensor_msgs/JointState` | `position[i]` | 姿态 / HOLD 参考 |
| `sensor_msgs/JointState` | `velocity[i]` | settle / capture 门槛 |
| `sensor_msgs/JointState` | `effort[i]` | \(\tau_{\mathrm{meas}}\) |
| `HardwareStatus` | `mode` | 确认 HOLD/TRACK |
| `HardwareStatus` | `last_command_sequence` | 链路健康 |
| `HardwareStatus` | `active_joint_mask` | 与 Goal.joint_mask 交叉检查 |
| `HardwareStatus` | `fault_code` | 见上表 |

**不要求**固件新增辨识专用 msg；若 executor 后续加 `IdentificationStatus`，须 backward-compatible，本设计不依赖。

### 4.6 其它安全

- **ARMED 前置**：`fault_code==0`、夹爪已锁、目标控制器已激活、与 `gravity_compensation_controller` / `joint4_calibration_controller` 互斥切换完成。
- **Gripper lock**：`lock_gripper=true` 时夹爪 `MODE_HOLD` 在固定开合，会话结束前不 TRACK。
- **Effort / vel limits**：激励与 HOLD 校正均受 `effort_limits` 与 Goal 上限约束；超限 → fault → HOLD。
- **人工支持**：第一版激励幅度保守；文档要求急停就绪（与现 Joint4 标定相同）。

---

## 5. 姿态激励草图

全臂重力（`mode=FULL_ARM_GRAVITY`）：

1. 使用预录 / 几何生成的 \(N \ge 2 \times n_{\mathrm{dof}}\) 组静态姿态（默认 N≈12），使各关节重力力矩符号与幅值充分变化。
2. 每姿态：`MOVING`（限速 TRACK）→ `HOLD_SETTLE`（`settle_time_s`，\|q̇\| &lt; 阈值）→ `CAPTURING`（窗内均值 \(\bar q, \bar\tau_{\mathrm{meas}}\)）。
3. 模型：\( \tau_{\mathrm{model}} = g(q) \)（现有 Pinocchio `GravityCompensation`）。
4. 拟合（每关节独立一阶）：  
   \(\tau_{\mathrm{meas},j} \approx s_j \, \tau_{\mathrm{model},j} + b_j\)  
   → `gravity_scales[j]=s_j`，`torque_biases_nm[j]=b_j`。
5. 对向接近同一姿态可减静摩擦偏置（可选第二遍）。

Payload 子模式见 §6。

---

## 6. 全臂重力 ID vs Payload 子模式

| | Full-arm gravity | Payload sub-mode（旧 Joint4） |
|--|------------------|-------------------------------|
| Goal.mode | 0 | 1 |
| 辨识量 | 六关节 `gravity_scales` + `torque_biases` | 主估计 camera_link 等效质量；附带 joint4 scale/bias |
| 激励 | 全臂多姿态 | 以 joint4 力矩跨度为主（兼容现 `begin_pose`/`capture`/`fit` 数据模型） |
| 控制器 | 新编排节点 + effort 前馈；可复用 GravityCompensation | 可调用现有 `Joint4CalibrationController` 服务作后端，或同一采样管线 + `compute_point_mass_gravity_sensitivity` |
| UI | 默认「一键全臂」 | 高级选项「负载 / Joint4」 |

旧手工 Trigger 流程保留；payload 子模式是其自动版，不是删除。

---

## 7. 工作拆分

### rebotarm-ros2-stack（本仓库）

- 定义 `GravityIdentification.action`；实现 `/StartGravityID` 编排节点（状态机、激励、拟合、YAML、热加载）。
- 控制器互斥切换；扩展 `GravityCompensationController` 支持 biases + reload。
- 文档（本文）与 launch / 参数默认值。
- **不**改固件协议字节布局，除非与 executor 同步 bump。

### rebotarm-executor（固件）

- 文档 `docs/ros2-interface.md`：确认并写明  
  - 辨识会话仅使用 **meas + HOLD**（及换姿 TRACK）；  
  - **`command_timeout_ms` → fault → HOLD**；  
  - **`fault_code` 表**（上表命名）；  
  - `JointCommand` / `JointState` / `HardwareStatus` 字段稳定。  
- 实现/核对：命令丢失锁存、HOLD 保持、effort 限幅；**不**实现辨识算法。
- 若需可配置超时：接受 host 侧参数或保持编译默认，并在接口文档写死默认值。

### robot-debug-desk（UI）

- 调用 `/StartGravityID`；绑定 Feedback：`phase`、`pose_index`/`pose_count`、`residual_nm`、`fault_code`、`message`。
- 展示 Result scales/biases/RMSE；提供保存 / 热加载开关（对应 Goal 字段）。
- Cancel → 依赖 action abort → HOLD 契约；UI 显示「已 HOLD」。

---

## 8. 与现状对照

| 现有 | 本设计 |
|------|--------|
| `GravityCompensation` + `gravity_scales` | 继续作为前馈与模型源；增加 biases + YAML 热加载 |
| `Joint4CalibrationController` 手工 Trigger | 保留；payload 子模式自动编排可复用其拟合思想 |
| 固件 MODE_HOLD/TRACK + HardwareStatus | 会话唯一依赖；明确 timeout / fault / abort→HOLD |

## 9. 开放问题（实现前拍板）

1. `torque_biases` 是否第一版就写入 GravityCompensationController（建议：是，默认 0）。
2. 激励姿态集：内置表 vs desk 下发 `pose_set_id`。
3. fault code 数值是否已在 executor 占用（以 executor 表为权威，本文名为契约）。
