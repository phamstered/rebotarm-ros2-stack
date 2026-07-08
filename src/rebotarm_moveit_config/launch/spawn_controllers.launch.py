"""
spawn_controllers.launch.py — 加载并激活 ros2_control 控制器

启动命令：
  ros2 launch rebotarm_moveit_config spawn_controllers.launch.py

用途：
  启动 ros2_control 控制器管理器，加载并激活 ros2_controllers.yaml 中定义的控制器：
  - arm_controller: JointTrajectoryController（6 轴臂轨迹控制）
  - joint_state_broadcaster: JointStateBroadcaster（关节状态广播）

使用场景：
  在 Gazebo 或 mock_components 环境启动后，需要显式加载控制器才能接收轨迹指令。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
