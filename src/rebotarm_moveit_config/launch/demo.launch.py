"""
demo.launch.py — MoveIt 独立演示启动

启动命令：
  ros2 launch rebotarm_moveit_config demo.launch.py

用途：
  最简单的 MoveIt 演示入口。启动完整的 MoveIt 环境：
  - robot_state_publisher（机器人描述）
  - move_group（运动规划节点）
  - RViz2（带 MoveIt 插件的可视化界面）
  - fake controller（仿真控制器，无需真实硬件）

使用场景：
  快速验证 MoveIt 配置是否正确、测试运动规划功能。
  不连接 Gazebo 或真实硬件，使用 mock_components。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
