# CMakeLists.txt — rebotarm_moveit_config 构建配置
#
# 这是一个纯配置包（无编译代码），仅安装配置文件和启动脚本到 share 目录。
# 由 MoveIt Setup Assistant 自动生成。

cmake_minimum_required(VERSION 3.8)
project(rebotarm_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装 config/ 和 launch/ 目录到包的 share 目录
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# 安装 Setup Assistant 元数据文件
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME real_move_group_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_move_group.py)
endif()
ament_package()
