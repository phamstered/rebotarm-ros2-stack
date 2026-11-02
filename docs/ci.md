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
| `colcon test` | `microros_stm32h723_bridge`、`rebotarm_gazebo`、`rebotarm_moveit_config`（增益 / msg / launch 契约、相机 launch、真机 MoveIt 配置与 RobotModel 限位探针） |
| 隔离冒烟 | mock MCU 上的轨迹速度（`ROS_DOMAIN_ID=229`）与夹爪轨迹（`GRIPPER_SMOKE=1`、domain 230）；无执行的 MoveIt 全姿态 / 夹爪规划（`ROS_LOCALHOST_ONLY=1`、domain 226） |

`rosdep` 跳过仓库外键：`stararm102_description`、`micro_ros_agent`。

## CI 基础镜像

`.github/workflows/ci-image.yml` 在手动触发，或 `main` 上 `docker/ci/**`、任意 `package.xml`、该工作流本身变更时，构建并推送 `ghcr.io/phamstered/rebotarm-ros2-ci`（标签 `humble`、`humble-<short-sha>`，以及可选日期标签）。镜像只预装 Humble 与 `rosdep` 依赖，不含工作区源码，也不是真机运行镜像。细节见 `docker/ci/README.md`。

上面的 `ci.yml` **仍然**在 `ubuntu-22.04` 上用 `ros-tooling/setup-ros` 安装 Humble，还没有切到该镜像。不要把 GHCR 标签当成当前 CI 的实际运行环境。

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
