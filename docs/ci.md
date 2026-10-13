# CI 说明

工作流：`.github/workflows/ci.yml`（风格对齐 `robot-debug-desk`）。

## 触发

- `push` / `pull_request` → `main`
- Runner：`ubuntu-22.04`，ROS 2 Humble（`ros-tooling/setup-ros`）

## 覆盖范围

| 覆盖 | 说明 |
|------|------|
| `colcon build` | `src/` 下包（含嵌套 `src/plugin/microros_stm32h723_bridge`） |
| pytest | `joint_state_relay/test`（配置与夹爪映射辅助） |
| ament pytest | `microros_stm32h723_bridge`（配置 / 源码契约） |

`rosdep` 跳过仓库外键：`stararm102_description`、`micro_ros_agent`。

## 不覆盖（需真机 / GUI）

- `start_jetson.sh`、CAN、STM32 micro-ROS agent 上电
- 实时 Gazebo / MoveIt 演示
- 外部 overlay 包

新行为应先补上述测试，再本地 `colcon test` 与对应 launch 冒烟。硬件改动请记录设备、ROS domain 与现象。

## 文档站点工作流

`.github/workflows/deploy-docs.yml` 在 `main` push 或 `workflow_dispatch` 时构建 VitePress 并部署到 GitHub Pages。

**设置（合并后由仓库管理员确认）：**

1. **Settings → Pages → Source** = **GitHub Actions**
2. 若仓库为 **private**，GitHub Pages 需要 **GitHub Pro**（或同等付费能力）；否则 Actions 可能成功但站点不可用
3. 首次合并后若未自动部署，在 Actions 中手动 Run **Deploy docs to GitHub Pages** 一次

站点预期 URL：`https://phamstered.github.io/rebotarm-ros2-stack/`（`base` 已设为 `/rebotarm-ros2-stack/`）。
