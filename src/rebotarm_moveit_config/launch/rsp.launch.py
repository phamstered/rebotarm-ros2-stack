import os
import xacro
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rebotarm_moveit_config")
    xacro_path = os.path.join(pkg_share, "config", "rebotarm_moveit.urdf.xacro")
    gazebo_pkg = get_package_share_directory("rebotarm_gazebo")
    meshes_dir = os.path.join(gazebo_pkg, "meshes")

    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)
    robot_description = re.sub(r"<!--(.*?)-->", "", doc.toxml(), flags=re.DOTALL)
    robot_description = robot_description.replace(
        "package://rebotarm_gazebo/meshes/", meshes_dir + "/"
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
