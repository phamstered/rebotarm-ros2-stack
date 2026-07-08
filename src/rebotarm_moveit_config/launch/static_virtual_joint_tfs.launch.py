"""
static_virtual_joint_tfs.launch.py — 发布 world → base_link 静态变换

启动命令：
  ros2 launch rebotarm_moveit_config static_virtual_joint_tfs.launch.py

用途：
  发布一个零位移/零旋转的静态 TF 变换：world → base_link。
  对应 SRDF 中定义的 virtual_joint，将机器人锚定在世界坐标系中。

变换内容：
  平移: (0, 0, 0)
  旋转: (0, 0, 0) — 四元数或欧拉角，均为零
  父帧: world
  子帧: base_link

注意：
  如果机器人 base_link 已通过其他方式（如 Gazebo 模型 spawn）定位到世界坐标系，
  此变换可能产生冲突。需根据实际场景决定是否启用。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            output="log",
        ),
    ])
