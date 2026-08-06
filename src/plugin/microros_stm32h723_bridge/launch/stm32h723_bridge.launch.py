from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('microros_stm32h723_bridge')
    xacro_file = PathJoinSubstitution(
        [package_share, 'urdf', 'rebotarm.micro_ros.xacro'])
    controllers_file = PathJoinSubstitution(
        [package_share, 'config', 'ros2_controllers.yaml'])
    gains_file = PathJoinSubstitution(
        [package_share, 'config', 'mit_gains.yaml'])

    serial_device = LaunchConfiguration('serial_device')
    baudrate = LaunchConfiguration('baudrate')
    start_agent = LaunchConfiguration('start_agent')
    required_joint_mask = LaunchConfiguration('required_joint_mask')

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' gains_file:=', gains_file,
            ' required_joint_mask:=', required_joint_mask,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('required_joint_mask', default_value='0x7f'),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', serial_device, '-b', baudrate],
            output='screen',
            condition=IfCondition(start_agent),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers_file],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
