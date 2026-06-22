import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    relay_dir = get_package_share_directory('joint_state_relay')
    config_file = os.path.join(relay_dir, 'config', 'relay.yaml')

    leader_arm_dir = get_package_share_directory('stararm102_description')
    leader_urdf = os.path.join(leader_arm_dir, 'urdf', 'stararm102_description.urdf')

    with open(leader_urdf, 'r') as f:
        leader_robot_description = f.read()

    leader_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='leader_robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': leader_robot_description,
        }],
    )

    relay_node = Node(
        package='joint_state_relay',
        executable='relay_node',
        name='joint_state_relay',
        parameters=[{
            'config_path': config_file,
        }],
        output='screen',
    )

    gazebo_dir = get_package_share_directory('rebotarm_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'rebotarm_gazebo.launch.py')),
    )

    return LaunchDescription([
        leader_rsp,
        gazebo_launch,
        relay_node,
    ])
