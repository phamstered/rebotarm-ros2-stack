cmake_minimum_required(VERSION 3.8)
project(rebotarm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  rebotarm_gazebo/follower_visualizer.py
  rebotarm_gazebo/gz_joint_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
