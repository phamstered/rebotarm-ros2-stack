import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rebotarm_gazebo')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rebotarm.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'follower.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
            'publish_frequency': 15.0,
        }],
        output='screen')

    follower_visualizer = Node(
        package='rebotarm_gazebo',
        executable='follower_visualizer.py',
        name='follower_visualizer',
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen')

    return LaunchDescription([
        robot_state_publisher,
        follower_visualizer,
        rviz,
    ])
