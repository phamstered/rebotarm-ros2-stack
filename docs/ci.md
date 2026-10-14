# CI 说明

工作流：`.github/workflows/ci.yml`。

## 触发

- `push` / `pull_request` → `main`
- Runner：`ubuntu-22.04`，ROS 2 Humble（`ros-tooling/setup-ros`）

## 覆盖范围

| 覆盖 | 说明 |
|------|------|
| `colcon build` | `src/` 下全部包（含嵌套 `src/plugin/microros_stm32h723_bridge`） |
| pytest | `joint_state_relay/test`（`relay_microros_arm.yaml` 契约、夹爪 `linear_map`） |
| ament pytest | `microros_stm32h723_bridge/test`（增益长度、msg 契约、launch/源码字符串契约） |

`rosdep` 跳过仓库外键：`stararm102_description`、`micro_ros_agent`。

## 不覆盖（需真机 / GUI）

- `start_jetson.sh`、CAN、STM32 micro-ROS agent 上电
- 实时 Gazebo / MoveIt 演示
- 外部 overlay 包与 `reBotArm_control_py` 运行时依赖

新行为应先补上述测试，再本地 `colcon test` 与对应 launch 冒烟。硬件改动请记录设备、ROS domain 与现象。

## 文档站点工作流

`.github/workflows/deploy-docs.yml` 在 `main` push 或 `workflow_dispatch` 时构建 VitePress 并部署到 GitHub Pages。

**设置（仓库管理员）：**

1. **Settings → Pages → Source** = **GitHub Actions**
2. 若仓库为 **private**，GitHub Pages 需要付费能力
3. 首次合并后若未自动部署，在 Actions 中手动 Run **Deploy docs to GitHub Pages**

站点：`https://phamstered.github.io/rebotarm-ros2-stack/`（`base` = `/rebotarm-ros2-stack/`）。
