# MoveIt TRACK 增益测试基线（关闭重力补偿）

## 测试条件

- 日期：2026-08-30（Asia/Shanghai）
- 平台：Jetson + STM32H723 + `MicroRosSystem`
- 控制器：`joint1_trajectory_controller`（active）
- 遥操作控制器：`joint1_teleop_controller`（inactive）
- 命令模式：`mode=1`（TRACK）
- 重力补偿：关闭
- STM32 状态：`fault_code=0`，命令序列持续确认
- 误差定义：`position_error = desired - actual`

重力补偿配置：

```xml
<param name="enable_gravity_compensation">false</param>
```

当前 TRACK 增益与实际 `/stm32/joint_commands` 一致：

| 关节 | kp | kd | effort feed-forward |
|---|---:|---:|---:|
| joint1 | 20.0 | 3.0 | 0.0 |
| joint2 | 20.0 | 3.0 | 0.0 |
| joint3 | 20.0 | 3.0 | 0.0 |
| joint4 | 10.0 | 1.5 | 0.0 |
| joint5 | 10.0 | 1.5 | 0.0 |
| joint6 | 10.0 | 1.5 | 0.0 |

配置来源：`src/plugin/microros_stm32h723_bridge/config/single_joint_mit_gains.yaml`。

## 基线 A：MoveIt `Forward` 姿态

采样时间约为 2026-08-30 16:08:24+08:00。该姿态是 MoveIt 中的命名姿态 `Forward`，轨迹目标约为 joint2/joint3 各 `-1.57 rad`，也是本次 joint2/joint3 的主要承重测试姿态。

| 关节 | desired (rad) | actual (rad) | error (rad) | error (deg) |
|---|---:|---:|---:|---:|
| joint1 | 0.000021 | 0.030708 | -0.030688 | -1.76 |
| joint2 | -1.570010 | -1.993401 | +0.423391 | +24.26 |
| joint3 | -1.569990 | -1.310559 | -0.259430 | -14.86 |
| joint4 | -0.000021 | 0.123407 | -0.123429 | -7.07 |
| joint5 | 0.000068 | 0.013542 | -0.013474 | -0.77 |
| joint6 | -0.000046 | -0.018883 | +0.018837 | +1.08 |

重点基线：

- joint2：`kp=20.0, kd=3.0`，误差 `+0.423391 rad`（`+24.26°`）
- joint3：`kp=20.0, kd=3.0`，误差 `-0.259430 rad`（`-14.86°`）

## 测试方法

后续调整增益时只使用 MoveIt `Forward` 命名姿态，并保持负载、运动路径、到位后等待时间和采样窗口一致，记录峰值误差、稳定误差、振动、电流与温度。

采样命令：

```bash
ros2 topic echo /joint1_trajectory_controller/controller_state --once
ros2 topic echo /stm32/joint_commands --once
ros2 topic echo /stm32/status --once
```

## 试验 B：MoveIt `Forward`，joint1–3 `kp=30, kd=3`

采样时间：2026-08-30 16:52:39–16:53:15+08:00。

试验条件：

- 重力补偿仍关闭，effort feed-forward 为 0
- joint1–3：`kp=30.0, kd=3.0`
- joint4–6：`kp=10.0, kd=1.5`（未变）
- STM32：`mode=1`、`fault_code=0`
- `joint1_trajectory_controller`：active

首次采样：

| 关节 | desired (rad) | actual (rad) | error (rad) | error (deg) |
|---|---:|---:|---:|---:|
| joint1 | -0.000055 | 0.024223 | -0.024278 | -1.39 |
| joint2 | -1.570003 | -1.841954 | +0.271951 | +15.58 |
| joint3 | -1.570033 | -1.279659 | -0.290373 | -16.64 |
| joint4 | -0.000007 | 0.158885 | -0.158892 | -9.10 |
| joint5 | 0.000011 | 0.013542 | -0.013531 | -0.78 |
| joint6 | -0.000083 | -0.019264 | +0.019182 | +1.10 |

随后连续样本显示误差未稳定：

| 样本 | joint2 error (rad) | joint3 error (rad) | joint4 error (rad) |
|---:|---:|---:|---:|
| 1 | +0.271951 | -0.290373 | -0.158892 |
| 2 | +0.371516 | -0.251081 | -0.137147 |
| 3 | +0.633971 | -0.351027 | -0.205813 |
| 4 | +0.351680 | -0.210263 | -0.137529 |

四帧算术平均仅用于描述此次波动，不代表稳态误差：

- joint2：平均 `+0.407280 rad`（`+23.34°`），范围 `+0.271951～+0.633971 rad`，峰峰值 `0.362020 rad`（`20.74°`）
- joint3：平均 `-0.275686 rad`（`-15.80°`），范围 `-0.351027～-0.210263 rad`，峰峰值 `0.140764 rad`（`8.07°`）

与 `kp=20, kd=3` 的基线 A 相比，不能根据首次采样认定 joint2 已改善：joint2 四帧平均误差仅比基线小约 3.8%，但波动范围显著；joint3 四帧平均绝对误差反而增加约 6.3%。这组数据采集时机械臂尚未稳定，因此不作为最终静差；最终判断使用下方的标准化稳定复采样。

### 标准化稳定复采样

使用 `scripts/sample_moveit_tracking_error.py --settle 2 --duration 10 --interval 0.5` 重新采样，保留 20 帧。20 帧的各关节误差完全一致（标准差显示为 0），说明此次采样窗口内机械臂已经稳定；前述四帧的大范围变化应视为到达 Forward 后的过渡过程，不作为稳态误差。

| 关节 | desired mean (rad) | actual mean (rad) | error mean (rad) | error mean (deg) |
|---|---:|---:|---:|---:|
| joint1 | -0.000055 | 0.024223 | -0.024278 | -1.39 |
| joint2 | -1.570003 | -1.921683 | +0.351680 | +20.15 |
| joint3 | -1.570033 | -1.359770 | -0.210263 | -12.05 |
| joint4 | -0.000007 | 0.137522 | -0.137529 | -7.88 |
| joint5 | 0.000011 | -0.001717 | +0.001728 | +0.10 |
| joint6 | -0.000083 | -0.019264 | +0.019182 | +1.10 |

与 `kp=20, kd=3` 的 Forward 基线相比：

- joint2 绝对稳态误差从 `0.423391` 降至 `0.351680 rad`，减少 `0.071711 rad`，约改善 `16.9%`
- joint3 绝对稳态误差从 `0.259430` 降至 `0.210263 rad`，减少 `0.049167 rad`，约改善 `19.0%`

## 试验 C：MoveIt `Forward`，joint1–4 `kp=40`

使用 `scripts/sample_moveit_tracking_error.py --settle 2 --duration 10 --interval 0.5` 采样，保留 20 帧。20 帧误差完全一致，采样窗口内状态稳定。

试验条件：

- 重力补偿关闭，effort feed-forward 为 0
- joint1–3：`kp=40.0, kd=3.0`
- joint4：`kp=40.0, kd=1.5`
- joint5–6：`kp=10.0, kd=1.5`
- STM32：`mode=1`、`fault_code=0`、`active_joint_mask=0x7F`

| 关节 | desired mean (rad) | actual mean (rad) | error mean (rad) | error mean (deg) |
|---|---:|---:|---:|---:|
| joint1 | -0.000003 | -0.012016 | +0.012013 | +0.69 |
| joint2 | -1.569916 | -1.773289 | +0.203372 | +11.65 |
| joint3 | -1.569940 | -1.344893 | -0.225047 | -12.89 |
| joint4 | -0.000062 | 0.041009 | -0.041071 | -2.35 |
| joint5 | 0.000087 | -0.007820 | +0.007907 | +0.45 |
| joint6 | 0.000005 | -0.019264 | +0.019269 | +1.10 |

与试验 B（joint1–3 `kp=30`、joint4 `kp=10`）相比：

- joint1 绝对误差从 `0.024278` 降至 `0.012013 rad`，改善约 `50.5%`
- joint2 绝对误差从 `0.351680` 降至 `0.203372 rad`，改善约 `42.2%`
- joint3 绝对误差从 `0.210263` 增至 `0.225047 rad`，恶化约 `7.0%`
- joint4 绝对误差从 `0.137529` 降至 `0.041071 rad`，改善约 `70.1%`

与基线 A（joint1–3 `kp=20`、joint4 `kp=10`）相比：

- joint2 绝对误差改善约 `52.0%`
- joint3 绝对误差改善约 `13.3%`
- joint4 绝对误差改善约 `66.7%`
