cmake_minimum_required(VERSION 3.8)
project(microros_stm32h723_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Pinocchio 4 uses modern FindPython3, while Humble's rosidl_generator_py still
# consumes the legacy variable through python_cmake_module.
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "Python interpreter" FORCE)
find_package(pinocchio REQUIRED)
# Pinocchio 4 may leave PYTHON_SOABI with a leading dot. Humble's
# FindPythonExtra prepends another dot, producing an unimportable `..cpython`
# rosidl typesupport module. Recompute it from the selected system Python.
unset(PYTHON_SOABI)
unset(PYTHON_SOABI CACHE)
unset(PythonExtra_EXTENSION_SUFFIX CACHE)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/HardwareStatus.msg"
  "msg/VlaJointChunk.msg"
  DEPENDENCIES std_msgs
)

set(HARDWARE_LIBRARY ${PROJECT_NAME}_hardware)
add_library(${HARDWARE_LIBRARY} SHARED
  src/micro_ros_system.cpp
  src/plugin_export.cpp
)
target_include_directories(${HARDWARE_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${HARDWARE_LIBRARY}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(${HARDWARE_LIBRARY} yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${HARDWARE_LIBRARY} "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(hardware_interface microros_stm32h723_bridge.xml)

set(GRAVITY_CONTROLLER_LIBRARY ${PROJECT_NAME}_gravity_controller)
add_library(${GRAVITY_CONTROLLER_LIBRARY} SHARED
  src/gravity_compensation.cpp
  src/gravity_compensation_controller.cpp
  src/joint4_calibration_controller.cpp
  src/vla_position_controller.cpp
)
target_include_directories(${GRAVITY_CONTROLLER_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${GRAVITY_CONTROLLER_LIBRARY}
  ament_index_cpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
)
target_link_libraries(${GRAVITY_CONTROLLER_LIBRARY} pinocchio::pinocchio)
target_link_libraries(${GRAVITY_CONTROLLER_LIBRARY} "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(
  controller_interface gravity_compensation_controller.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(FILES
  microros_stm32h723_bridge.xml
  gravity_compensation_controller.xml
  DESTINATION share/${PROJECT_NAME})
install(TARGETS ${HARDWARE_LIBRARY} ${GRAVITY_CONTROLLER_LIBRARY}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/mock_stm32_node.py
  scripts/mock_vla_chunk_publisher.py
  scripts/single_motor_smoke_test.py
  scripts/stm32_state_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_teleop_checks test/test_teleop_checks.py)
  ament_add_gtest(test_stop_lifecycle test/test_stop_lifecycle.cpp)
  target_include_directories(test_stop_lifecycle PRIVATE include)
  target_link_libraries(test_stop_lifecycle ${HARDWARE_LIBRARY} "${cpp_typesupport_target}")
  ament_target_dependencies(test_stop_lifecycle hardware_interface rclcpp rclcpp_lifecycle sensor_msgs std_srvs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(${HARDWARE_LIBRARY} ${GRAVITY_CONTROLLER_LIBRARY})
ament_package()
