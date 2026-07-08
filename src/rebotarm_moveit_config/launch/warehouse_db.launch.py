"""
warehouse_db.launch.py — 启动 MoveIt Warehouse 数据库

启动命令：
  ros2 launch rebotarm_moveit_config warehouse_db.launch.py

用途：
  启动 MongoDB-backed 的 Warehouse 数据库，用于存储和检索：
  - 规划场景（planning scenes）
  - 机器人状态（robot states）
  - 运动计划（motion plans）

使用场景：
  需要持久化保存/加载规划场景时使用。一般演示和开发中不常用。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .to_moveit_configs()
    )
    return generate_warehouse_launch(moveit_config)
