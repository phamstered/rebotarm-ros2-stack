cmake_minimum_required(VERSION 3.8)
project(microros_stm32h723_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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
  DEPENDENCIES std_msgs
)

set(HARDWARE_LIBRARY ${PROJECT_NAME}_hardware)
add_library(${HARDWARE_LIBRARY} SHARED
  src/gravity_compensation.cpp
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
target_link_libraries(${HARDWARE_LIBRARY} pinocchio::pinocchio yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${HARDWARE_LIBRARY} "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(hardware_interface microros_stm32h723_bridge.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(FILES microros_stm32h723_bridge.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS ${HARDWARE_LIBRARY}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/mock_stm32_node.py
  scripts/single_motor_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(${HARDWARE_LIBRARY})
ament_package()
