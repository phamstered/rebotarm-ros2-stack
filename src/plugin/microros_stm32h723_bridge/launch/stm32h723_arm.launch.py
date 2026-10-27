from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    enable_joint4_calibration = LaunchConfiguration('enable_joint4_calibration')
    enable_gripper = LaunchConfiguration('enable_gripper')
    gravity_tuning = IfCondition(PythonExpression([
        "'", position_controller, "' == 'none' and ('",
        enable_gravity_controller, "'.lower() == 'true' or '",
        enable_joint4_calibration, "'.lower() == 'true') and '",
        state_only, "' == 'false'",
    ]))
    normal_control = IfCondition(PythonExpression([
        "not ('", position_controller, "' == 'none' and ('",
        enable_gravity_controller, "'.lower() == 'true' or '",
        enable_joint4_calibration, "'.lower() == 'true')) and '",
        state_only, "' == 'false'",
    ]))
    teleop_active = IfCondition(PythonExpression([
        "'", position_controller, "' == 'teleop' and '", state_only, "' == 'false'",
    ]))
    teleop_inactive = IfCondition(PythonExpression([
        "'", position_controller, "' == 'trajectory' and '", state_only, "' == 'false'",
    ]))
    trajectory_active = IfCondition(PythonExpression([
        "'", position_controller, "' == 'trajectory' and '", state_only, "' == 'false'",
    ]))
    trajectory_inactive = IfCondition(PythonExpression([
        "'", position_controller, "' == 'teleop' and '", state_only, "' == 'false'",
    ]))
    gravity_active = IfCondition(PythonExpression([
        "'", enable_gravity_controller, "'.lower() == 'true' and '",
        enable_joint4_calibration, "'.lower() != 'true' and '",
        position_controller, "' != 'none' and '",
        state_only, "' == 'false'",
    ]))
    gravity_inactive = IfCondition(PythonExpression([
        "('", enable_gravity_controller, "'.lower() != 'true' or '",
        position_controller, "' == 'none' or '",
        enable_joint4_calibration, "'.lower() == 'true') and '",
        state_only, "' == 'false'",
    ]))
    calibration_inactive = IfCondition(PythonExpression([
        "'", state_only, "' == 'false'",
    ]))
    gripper_active = IfCondition(PythonExpression([
        "'", enable_gripper, "'.lower() == 'true' and '",
        state_only, "' == 'false'",
    ]))

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' gains_file:=', gains_file,
            ' enable_gripper:=', enable_gripper,
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
            'enable_joint4_calibration',
            default_value='false',
            description=(
                'Load joint4 calibration mode for explicit activation by the '
                'guarded startup helper.'),
        ),
        DeclareLaunchArgument(
            'enable_gripper',
            default_value='false',
            description=(
                'Export and activate the calibrated gripper position path.'),
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
            condition=normal_control,
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_file,
                {
                    'use_sim_time': use_sim_time,
                    'hardware_components_initial_state': {
                        'inactive': ['Stm32H723ArmSystem'],
                    },
                },
            ],
            output='screen',
            condition=gravity_tuning,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time},
            ],
            # The ros2_control broadcaster can keep publishing its last joint
            # positions after the hardware component becomes unconfigured.
            # TF must follow the same live feedback used by Robot View.
            remappings=[('/joint_states', '/stm32/joint_states')],
            output='screen',
        ),
        Node(
            package='microros_stm32h723_bridge',
            executable='stm32_state_republisher.py',
            name='stm32_state_republisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'include_gripper': enable_gripper,
                'gripper_position_scale': -0.008612826752,
                'gripper_position_offset': 0.004282757541,
            }],
            output='screen',
            condition=IfCondition(state_only),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'gripper_teleop_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=gripper_active,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=normal_control,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=gravity_tuning,
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
                'joint4_calibration_controller', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=calibration_inactive,
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
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'vla_position_controller', '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            condition=normal_control,
        ),
    ])
