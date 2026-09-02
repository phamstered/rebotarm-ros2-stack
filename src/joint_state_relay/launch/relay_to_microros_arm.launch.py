from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('joint_state_relay'),
        'config',
        'relay_microros_arm.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        Node(
            package='joint_state_relay',
            executable='relay_node',
            name='arm_teleop_relay',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
            }],
            output='screen',
        ),
    ])
