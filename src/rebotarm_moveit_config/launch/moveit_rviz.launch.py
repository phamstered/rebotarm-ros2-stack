"""
moveit_rviz.launch.py — 启动带 MoveIt 插件的 RViz2

启动命令：
  ros2 launch rebotarm_moveit_config moveit_rviz.launch.py

用途：
  单独启动 RViz2，预加载 MoveIt 可视化配置。
  适用于 move_group 和仿真已单独运行，只需打开可视化界面的场景。

功能：
  - 加载 moveit.rviz 配置文件（包含 MoveIt 插件布局）
  - 传递规划管线和运动学参数给 RViz
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
