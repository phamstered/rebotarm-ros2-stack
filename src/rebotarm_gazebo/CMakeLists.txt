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

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME left_wrist_camera_launch
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_left_wrist_camera_launch.py
  )
endif()

ament_package()
