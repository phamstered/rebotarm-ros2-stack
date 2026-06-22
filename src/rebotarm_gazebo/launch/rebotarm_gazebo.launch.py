import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro
import re


def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def generate_launch_description():
    pkg_share = get_package_share_directory('rebotarm_gazebo')
    xacro_path = os.path.join(pkg_share, 'urdf', 'rebotarm.urdf.xacro')
    meshes_dir = os.path.join(pkg_share, 'meshes')
    models_dir = os.path.join(pkg_share, 'models')
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)
    robot_description = remove_comments(doc.toxml())
    robot_description = robot_description.replace(
        'package://rebotarm_gazebo/meshes/', meshes_dir + '/')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo in headless mode')

    ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.pathsep.join([
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            models_dir,
        ]))

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[
            ('gz_args', ['-r -v 4 empty.sdf']),
            ('gz_version', ['6']),
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'publish_frequency': 15.0,
        }],
        output='screen')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'rebotarm', '-allow_renaming', 'true'],
        output='screen')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]))

    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]))

    return LaunchDescription([
        headless_arg,
        ign_resource_path,
        gz_sim_launch,
        robot_state_publisher,
        spawn_entity,
        bridge,
        after_spawn,
        after_jsb,
    ])
