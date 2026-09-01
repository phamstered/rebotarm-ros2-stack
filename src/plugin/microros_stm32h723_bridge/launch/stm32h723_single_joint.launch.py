from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('microros_stm32h723_bridge')
    xacro_file = PathJoinSubstitution(
        [package_share, 'urdf', 'rebotarm.single_joint.micro_ros.xacro'])
    controllers_file = PathJoinSubstitution(
        [package_share, 'config', 'single_joint_controllers.yaml'])
    default_gains_file = PathJoinSubstitution(
        [package_share, 'config', 'single_joint_mit_gains.yaml'])
    gains_file = LaunchConfiguration('gains_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    state_only = LaunchConfiguration('state_only')
    controller_mode = LaunchConfiguration('controller_mode')
    teleop_mode = IfCondition(PythonExpression([
        "'", controller_mode, "' == 'teleop' and '", state_only, "' == 'false'",
    ]))
    gravity_mode = IfCondition(PythonExpression([
        "'", controller_mode, "' == 'gravity' and '", state_only, "' == 'false'",
    ]))

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' gains_file:=', gains_file,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'gains_file',
            default_value=default_gains_file,
            description='MIT gain profile loaded by MicroRosSystem.',
        ),
        DeclareLaunchArgument(
            'controller_mode',
            default_value='teleop',
            description='Active command controller: teleop or gravity.',
        ),
        DeclareLaunchArgument(
            'state_only',
            default_value='false',
            description=(
                'Publish STM32 feedback and robot description without '
                'starting ros2_control or command controllers.'),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_file,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
            condition=UnlessCondition(state_only),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='microros_stm32h723_bridge',
            executable='stm32_state_republisher.py',
            name='stm32_state_republisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(state_only),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=UnlessCondition(state_only),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'gravity_compensation_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=gravity_mode,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint1_trajectory_controller',
                '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=UnlessCondition(state_only),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint1_teleop_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=teleop_mode,
        ),
    ])
