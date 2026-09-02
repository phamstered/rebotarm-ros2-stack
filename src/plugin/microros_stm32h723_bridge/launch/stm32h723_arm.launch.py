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
        [package_share, 'urdf', 'rebotarm.arm.micro_ros.xacro'])
    controllers_file = PathJoinSubstitution(
        [package_share, 'config', 'arm_controllers.yaml'])
    default_gains_file = PathJoinSubstitution(
        [package_share, 'config', 'arm_mit_gains.yaml'])
    gains_file = LaunchConfiguration('gains_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    state_only = LaunchConfiguration('state_only')
    position_controller = LaunchConfiguration('position_controller')
    enable_gravity_controller = LaunchConfiguration('enable_gravity_controller')
    teleop_active = IfCondition(PythonExpression([
        "'", position_controller, "' == 'teleop' and '", state_only, "' == 'false'",
    ]))
    teleop_inactive = IfCondition(PythonExpression([
        "'", position_controller, "' != 'teleop' and '", state_only, "' == 'false'",
    ]))
    trajectory_active = IfCondition(PythonExpression([
        "'", position_controller, "' == 'trajectory' and '", state_only, "' == 'false'",
    ]))
    trajectory_inactive = IfCondition(PythonExpression([
        "'", position_controller, "' != 'trajectory' and '", state_only, "' == 'false'",
    ]))
    gravity_active = IfCondition(PythonExpression([
        "'", enable_gravity_controller, "'.lower() == 'true' and '",
        state_only, "' == 'false'",
    ]))
    gravity_inactive = IfCondition(PythonExpression([
        "'", enable_gravity_controller, "'.lower() != 'true' and '",
        state_only, "' == 'false'",
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
            'position_controller',
            default_value='teleop',
            description='Active position controller: teleop, trajectory, or none.',
            choices=['teleop', 'trajectory', 'none'],
        ),
        DeclareLaunchArgument(
            'enable_gravity_controller',
            default_value='false',
            description='Activate the independent effort-only gravity controller.',
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
            condition=gravity_active,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'gravity_compensation_controller', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=gravity_inactive,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'arm_trajectory_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=trajectory_active,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'arm_trajectory_controller', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=trajectory_inactive,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'arm_teleop_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=teleop_active,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'arm_teleop_controller', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=teleop_inactive,
        ),
    ])
