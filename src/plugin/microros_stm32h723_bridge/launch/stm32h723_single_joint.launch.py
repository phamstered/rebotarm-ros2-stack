from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('microros_stm32h723_bridge')
    xacro_file = PathJoinSubstitution(
        [package_share, 'urdf', 'rebotarm.single_joint.micro_ros.xacro'])
    controllers_file = PathJoinSubstitution(
        [package_share, 'config', 'single_joint_controllers.yaml'])
    gains_file = PathJoinSubstitution(
        [package_share, 'config', 'single_joint_mit_gains.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' gains_file:=', gains_file,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_file,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint1_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
        # Load the streaming teleoperation controller without claiming joint1.
        # Switch from joint1_controller explicitly before starting teleoperation.
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint1_teleop_controller',
                '--inactive',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
    ])
