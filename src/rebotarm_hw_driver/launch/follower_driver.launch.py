import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rebotarm_gazebo_dir = get_package_share_directory('rebotarm_gazebo')

    default_urdf_path = os.path.join(rebotarm_gazebo_dir, 'urdf', 'rebotarm.urdf')
    default_rviz_path = os.path.join(rebotarm_gazebo_dir, 'rviz', 'rebotarm.rviz')

    with open(default_urdf_path, 'r') as f:
        robot_description_content = f.read()

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    rate_arg = DeclareLaunchArgument('rate', default_value='50.0')
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/follower/joint_states',
        description='Topic name for follower joint states (avoids collision with leader)')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    follower_driver_node = Node(
        package='rebotarm_hw_driver',
        executable='follower_driver',
        name='rebotarm_follower_driver',
        parameters=[{
            'rate': LaunchConfiguration('rate'),
        }],
        remappings=[
            ('joint_states', LaunchConfiguration('output_topic')),
        ],
        output='screen',
        sigterm_timeout='120',
        sigkill_timeout='10',
    )

    return LaunchDescription([
        use_rviz_arg,
        rate_arg,
        output_topic_arg,
        robot_state_publisher_node,
        rviz_node,
        follower_driver_node,
    ])
