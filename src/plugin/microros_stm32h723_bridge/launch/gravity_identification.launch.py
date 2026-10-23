"""Launch /StartGravityID orchestrator next to STM32 arm bring-up.

Recommended pairing on Jetson (exclusive bus):
  1) Bring up micro-ROS agent + arm in state_only (or stop CM writers) so
     MicroRosSystem is not publishing /stm32/joint_commands.
  2) ros2 launch microros_stm32h723_bridge gravity_identification.launch.py
  3) ros2 action send_goal /StartGravityID \\
       microros_stm32h723_bridge/action/GravityIdentification \\
       "{joint_mask: 0, mode: 0, write_yaml: true, hot_reload: false, lock_gripper: true}"

Abort → HOLD (not DISABLE). PAYLOAD mode (1) is rejected in v1.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('microros_stm32h723_bridge')
    default_params = PathJoinSubstitution(
        [package_share, 'config', 'gravity_identification.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter YAML for gravity_identification node.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        Node(
            package='microros_stm32h723_bridge',
            executable='gravity_identification_node',
            name='gravity_identification',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
