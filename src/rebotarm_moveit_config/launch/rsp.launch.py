"""
rsp.launch.py — Robot State Publisher 启动（自定义实现）

启动命令：
  ros2 launch rebotarm_moveit_config rsp.launch.py

用途：
  启动 robot_state_publisher 节点，发布机器人 TF 变换。

与标准实现的区别：
  这是自定义实现而非 MoveItConfigsBuilder 的内置方法，因为需要：
  1. 处理 URDF xacro 文件
  2. 剥离 XML 注释（避免某些解析器问题）

注意：
  - robot_state_publisher 会根据 URDF 中的 joint 定义，
    持续发布各 link 之间的 TF 变换树。
  - mesh 文件使用 package:// URL，依赖 ament_index 解析。
    确保 rebotarm_gazebo 包已正确安装并 source。
"""

import os
import xacro
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rebotarm_moveit_config")
    xacro_path = os.path.join(pkg_share, "config", "rebotarm_moveit.urdf.xacro")

    # 处理 xacro → URDF
    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)

    # 剥离 XML 注释（避免某些解析器的兼容性问题）
    robot_description = re.sub(r"<!--(.*?)-->", "", doc.toxml(), flags=re.DOTALL)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
