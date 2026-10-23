# 一键全关节重力 / 参数辨识（设计草案）

> 房间共识摘要：编排在本仓库（ROS）；固件（rebotarm-executor）只做 **meas + HOLD/TRACK**（「重力辨识会话」，无 GravityID 模式位）；UI（rebotarm-debug-desk）只调 action。  
> 状态：**Action 接口已冻结**（PR #23）；**FULL_ARM 编排已落地**（`gravity_identification_node` → `/StartGravityID`）。PAYLOAD 子模式 / 命名姿态集 / Jetson E2E 仍为后续。现有 `GravityCompensationController`（含 `torque_biases` + 参数热加载）与 `Joint4CalibrationController` 仍保留。

## 1. Goal / Non-goals

### Goal

- 提供 **一次 action 调用** 完成选中关节的重力缩放 / 力矩偏置辨识（可推广到全臂，不限于 joint4）。
- 动作名（服务器名）：**`/StartGravityID`**；类型名：`microros_stm32h723_bridge/action/GravityIdentification`。
- 输出可写入 YAML，并支持热加载进 `GravityCompensationController`。
- 安全：会话内固件仅用现有 **HOLD / TRACK**（无 GravityID 模式位）；abort / 命令丢失 → **HOLD**（不是 DISABLE）；夹爪锁定；effort / 速度限幅。
- 明确 **固件接口契约**（「重力辨识会话」、命令超时、fault code 十进制现网值、反馈字段），与 executor `docs/ros2-interface.md` 对齐。

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


### 2.5 固件侧「重力辨识会话」语义（对齐 executor）

上位编排在固件侧 **不**增加 GravityID 模式位或专用消息。辨识会话 = 一串合法的 **HOLD / TRACK** `JointCommand` + 照常发布的 `JointState` / `HardwareStatus`（与 [rebotarm-executor PR #7](https://github.com/phamstered/rebotarm-executor/pull/7) / `docs/ros2-interface.md`「重力辨识会话」一致）。

ROS 编排节点在会话中只要求固件：

1. **持续接受并执行 `JointCommand`**（与正常 HOLD/TRACK 相同频率，不得停发）。
2. **测量**：照常发布 `JointState`（position / velocity / effort）与 `HardwareStatus`。
3. **不**实现 `begin_pose` / `fit` 等服务；那些留在 ROS 或被 action 取代。

阶段约定：

| 阶段 | 固件期望 | 说明 |
|------|----------|------|
| 换姿激励 | 短暂 `MODE_TRACK`（1） | 到位后 host 立即切回静置策略 |
| 静置 / 采样（无前馈） | `MODE_HOLD`（0） | 首次进入 HOLD 时固件锁存实测位置；发送速度 0、**力矩 0**，沿用命令 kp/kd |
| 静置 / 采样（需重力前馈） | **`MODE_TRACK` @ velocity≈0**，`effort[]` = 重力前馈 | **现网 HOLD 会清零 MIT effort**；不要假设 HOLD 能保持重力前馈。若未来固件提供 HOLD-with-effort，再同步 bump 本文与 executor 接口文档 |
| Abort / cancel | host 下发 HOLD | **不**自动 DISABLE；DISABLE 仅走现有 `safe_stop` / 反馈超时等路径 |

固件无需识别「辨识模式」枚举；超时 / 命令丢失走现网 **command-loss → HOLD** 路径（见 §4.3）。

---

### 2.6 Desk contract（字段冻结）

Desk / UI 可立即按下列字段绑定；**不要**等待编排状态机落地。完整 IDL 以包内 `action/GravityIdentification.action` 为准。

| 分区 | 字段 |
|------|------|
| **Goal** | `joint_mask`, `mode`, `pose_set_id`, `settle_time_s`, `capture_window_s`, `write_yaml`, `hot_reload`, `output_yaml_path`, `max_effort_nm`, `max_velocity_rad_s`, `command_timeout_ms`, `lock_gripper` |
| **Feedback** | `phase`, `pose_index`, `pose_count`, `active_joint_mask`, `residual_nm`, `measured_effort_nm`, `model_effort_nm`, `rmse_running`, `fault_code`, `message` |
| **Result** | `success`, `fault_code`, `gravity_scales`, `torque_biases_nm`, `rmse_per_joint`, `rmse_total`, `yaml_path`, `message` |

常量（亦写入 `.action` IDL 与 `include/.../gravity_identification_constants.hpp`）：

- `mode`: `MODE_FULL_ARM_GRAVITY=0`（默认路径）, `MODE_PAYLOAD_SUBMODE=1`
- `phase`: `PHASE_IDLE=0` … `PHASE_FAILED=10`（见 §4.1）
- `joint_mask==0` → 默认全臂 `DEFAULT_JOINT_MASK=0x3F`

服务器名：`/StartGravityID`。编排节点未实现时，stub（若存在）会拒绝 goal 并返回 `success=false`。

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
| 4 | `PHASE_HOLD_SETTLE` | 静置（HOLD，或需前馈时 TRACK@v≈0） |
| 5 | `PHASE_CAPTURING` | 采样窗（同上；现网 HOLD 清零 effort） |
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
2. Host 对所有活动关节下发 **`JointCommand.mode = MODE_HOLD (0)`**，`position = 当前实测位置`，`velocity = 0`（现网 HOLD 下 MIT effort 为 0；abort 优先进 HOLD，不是 DISABLE）。
3. 夹爪保持锁定 HOLD。
4. 不自动 DISABLE；DISABLE 仅走现有 `safe_stop` / 反馈超时等硬件保护路径。
5. Feedback.phase = `PHASE_ABORTING`，然后 Result.success=false。

### 4.3 固件命令超时（command timeout）— executor 必写进 ros2-interface

| 字段 / 行为 | 约定 |
|-------------|------|
| 命令流 | Host 在会话内按现有桥接频率持续发 `JointCommand`（与正常 HOLD/TRACK 相同） |
| **`command_timeout_ms`** | 固件在 **最后一帧合法 JointCommand** 之后若超过此时长未收到新命令 → 判定命令丢失 |
| 默认建议 | Host 期望对齐约 **100 ms**（与桥 `state_timeout_ms: 100` / 固件 `COMMAND_TIMEOUT_MS` 同量级）；现网超时不可由 Goal 运行时改写 |
| 超时后固件行为（现网） | 与 executor `COMMAND_TIMEOUT_MS`（100，TRACK→HOLD、不置故障）及 `COMMAND_LOSS_LATCH_MS`（500 → 锁存 **fault_code = 11**，若已使能则 **HOLD**，否则 DISABLE）一致；会话 abort 契约取 **HOLD**，不是 DISABLE |
| Host 行为 | 读到 fault **11** → action ABORTING → 若链路仍通可继续发 HOLD；否则引导 `safe_stop` |

> 固件 **不**增加 GravityID 模式位。辨识会话依赖现网 **command-loss → HOLD** 路径（见 executor「重力辨识会话」）；desk/编排不得在会话中停发命令。Goal.`command_timeout_ms` 为 host 侧期望对齐值；现网超时为编译期常量，固件不接受运行时改写。

### 4.4 Fault codes（`HardwareStatus.fault_code`，uint16）

**权威为 executor 现网十进制枚举**（`MotorBridgeFault`，不是位标志）。下列草案名仅作别名；**不得**再使用 `0x0001` 风格虚构十六进制位。数值以 [rebotarm-executor PR #7](https://github.com/phamstered/rebotarm-executor/pull/7) / `docs/ros2-interface.md` 为准。

| 现网值 | 草案别名 | 现网符号 | 谁置位 | 会话内 ROS 动作 |
|-------:|----------|----------|--------|-----------------|
| 0 | `FAULT_NONE` | `MOTOR_FAULT_NONE` | — | 继续 |
| **11** | `FAULT_CMD_LOSS` | `MOTOR_FAULT_COMMAND_LOSS_LATCHED` | 固件 | abort → HOLD |
| **4** | `FAULT_STALE_SEQUENCE` | `MOTOR_FAULT_STALE_SEQUENCE` | 固件 | abort；按现网清序列/重启策略 |
| **7** | `FAULT_MOTOR` | `MOTOR_FAULT_DRIVE` | 固件 | abort → HOLD；禁止继续激励（驱动故障现网常伴随 DISABLE） |
| **3**（近义） | `FAULT_EFFORT_LIMIT` | `MOTOR_FAULT_COMMAND_LIMIT` | 固件 | abort → HOLD；现网为收命令时 kp/kd/\|effort\| 超限，非运行中力矩限幅专用码 |

以下 **留在 ROS 侧**（或未来扩展）；**勿占用**固件 0–12：

| 草案名 | 现网值 | 谁置位 | 会话内 ROS 动作 |
|--------|--------|--------|-----------------|
| `FAULT_VELOCITY_LIMIT` | — | ROS | abort → HOLD |
| `FAULT_POSITION_DEVIATION` | — | ROS | 本姿态作废或整次 abort（可配置） |
| `FAULT_GRIPPER_UNLOCKED` | — | ROS | abort（若要求 lock_gripper） |
| `FAULT_SESSION_REJECTED` | — | ROS | 不进入 ARMED；现网无「拒绝过宽 timeout」码 |

ROS Result/Feedback 的 `fault_code` **原样透传** `HardwareStatus.fault_code`；ROS 自检故障可用高位（例如 `0x8000 | local_code`）并在 `message` 说明，避免与固件 0–12 冲突。现网另有码（1、2、5、6、8、9、10、12）语义不变；会话中出现时同样应停止激励并按安全策略处理。

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
- **Effort / vel limits**：激励与静置（HOLD 或 TRACK@v≈0）均受 `effort_limits` 与 Goal 上限约束；超限 → fault → HOLD。
- **人工支持**：第一版激励幅度保守；文档要求急停就绪（与现 Joint4 标定相同）。

---

## 5. 姿态激励草图

全臂重力（`mode=FULL_ARM_GRAVITY`）：

1. 使用预录 / 几何生成的 \(N \ge 2 \times n_{\mathrm{dof}}\) 组静态姿态（默认 N≈12），使各关节重力力矩符号与幅值充分变化。
2. 每姿态：`MOVING`（限速 TRACK）→ `HOLD_SETTLE`（`settle_time_s`，\|q̇\| &lt; 阈值；无前馈用 HOLD，需前馈用 TRACK@v≈0）→ `CAPTURING`（窗内均值 \(\bar q, \bar\tau_{\mathrm{meas}}\)）。
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

- 文档 `docs/ros2-interface.md`「重力辨识会话」（见 PR #7）：  
  - 会话仅用 **meas + HOLD/TRACK**，**无** GravityID 模式位；  
  - 命令丢失 → **HOLD**（不是会话 abort 首选 DISABLE）；  
  - **`fault_code` 现网十进制表**（CMD_LOSS=11，STALE_SEQUENCE=4，MOTOR=7 等）；  
  - 现网 HOLD 清零 MIT effort；前馈用 TRACK@v≈0（或未来 HOLD-with-effort）；  
  - `JointCommand` / `JointState` / `HardwareStatus` 字段稳定。  
- 实现/核对：命令丢失锁存、HOLD 保持、effort 限幅；**不**实现辨识算法。
- 超时为编译期常量（`COMMAND_TIMEOUT_MS` / `COMMAND_LOSS_LATCH_MS`）；不接受 Goal 运行时改写。

### rebotarm-debug-desk（UI）

- 调用 `/StartGravityID`；绑定 Feedback：`phase`、`pose_index`/`pose_count`、`residual_nm`、`fault_code`、`message`。
- 展示 Result scales/biases/RMSE；提供保存 / 热加载开关（对应 Goal 字段）。
- Cancel → 依赖 action abort → HOLD 契约；UI 显示「已 HOLD」。

---

## 8. 与现状对照

| 现有 | 本设计 |
|------|--------|
| `GravityCompensation` + `gravity_scales` | 继续作为前馈与模型源；增加 biases + YAML 热加载 |
| `Joint4CalibrationController` 手工 Trigger | 保留；payload 子模式自动编排可复用其拟合思想 |
| 固件 MODE_HOLD/TRACK + HardwareStatus | 会话唯一依赖（「重力辨识会话」）；command-loss→HOLD；fault 用现网十进制 |

## 9. 开放问题（实现前拍板）

1. `torque_biases` 是否第一版就写入 GravityCompensationController（建议：是，默认 0）。
2. 激励姿态集：内置表 vs desk 下发 `pose_set_id`。
3. ~~fault code 数值是否已在 executor 占用~~ — **已拍板**：以 executor 现网十进制为准（见 §4.4；别名映射见 executor PR #7）。
4. 静置/采样是否第一版就强制 TRACK@v≈0 前馈，还是允许纯 HOLD（effort=0）采样后再拟合（建议：有 GravityCompensation 时用 TRACK@v≈0）。

---

## 10. 实现状态（v1 FULL_ARM）

| 项 | 状态 |
|----|------|
| Action IDL `/StartGravityID` | 已合入 main（PR #23） |
| 编排节点 `gravity_identification_node` | **已实现**（本 PR）：IDLE→…→DONE/FAILED；连续 JointCommand；TRACK@v≈0 采样；abort→HOLD |
| 内置激励姿态 | 12 组；j1–3 / j4–6 分幅值（DM4340P vs DM4310） |
| 拟合 `τ≈s·τ_model+b` | 头文件纯函数 + gtest |
| YAML `~/.rebotarm/gravity_id_latest.yaml` | 已写 schema_version=1 |
| GC `torque_biases` + `set_parameters` 热加载 | 已实现；若控制器未激活则 soft-fail + 文档手动 reload |
| PAYLOAD_SUBMODE | **拒绝**（用现有 `joint4_calibration_controller`） |
| `pose_set_id` 命名集 | 忽略并 WARN（v1 仅内置集） |
| 与 MicroRosSystem 互斥 | 尝试 `switch_controller` stop；**推荐 `state_only` bring-up** 独占 `/stm32/joint_commands` |
| Jetson 实机 E2E | **未在 CI 盒验证** — 见 PR 说明 |

### 如何运行（Jetson）

```bash
# Terminal A: agent + arm read path (state_only avoids CM writing JointCommand)
ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py state_only:=true

# Terminal B: orchestrator
ros2 launch microros_stm32h723_bridge gravity_identification.launch.py

# Terminal C: desk / CLI
ros2 action send_goal /StartGravityID \
  microros_stm32h723_bridge/action/GravityIdentification \
  "{joint_mask: 0, mode: 0, write_yaml: true, hot_reload: true, lock_gripper: true, settle_time_s: 1.5, capture_window_s: 1.0, max_velocity_rad_s: 0.15, max_effort_nm: 10.0}"
```

热加载失败时：把 Result.`yaml_path` 中的 `gravity_scales` / `torque_biases_nm` 拷进 `arm_controllers.yaml` 后重新 activate `gravity_compensation_controller`，或对运行中的控制器调用 `ros2 param set /gravity_compensation_controller …`。

### 话题（与现网一致，无改名）

| 用途 | 话题 |
|------|------|
| 命令 | `/stm32/joint_commands` (`JointCommand`) |
| 采样 | `/stm32/joint_states` (`sensor_msgs/JointState`) |
| 故障透传 | `/stm32/status` (`HardwareStatus`) |
| Action | `/StartGravityID` |
