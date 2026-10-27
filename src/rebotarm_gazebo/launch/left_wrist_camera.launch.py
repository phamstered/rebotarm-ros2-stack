"""Launch the left-arm wrist RealSense with a robot-connected TF frame."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_namespace": "cameras/left_arm",
            "camera_name": "wrist",
            "tf_prefix": "left_arm/",
            "serial_no": serial_no,
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
        }.items(),
    )

    # The installed URDF connects camera_link to link6. The RealSense wrapper
    # constructs left_arm/wrist_link from camera_name + tf_prefix.
    camera_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_wrist_camera_mount",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "camera_link",
            "--child-frame-id", "left_arm/wrist_link",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="''",
            description="RealSense device serial number when multiple cameras are attached",
        ),
        camera_mount,
        camera,
    ])
