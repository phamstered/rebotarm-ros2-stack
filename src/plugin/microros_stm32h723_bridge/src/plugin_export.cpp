#include "hardware_interface/system_interface.hpp"
#include "microros_stm32h723_bridge/micro_ros_system.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  microros_stm32h723_bridge::MicroRosSystem,
  hardware_interface::SystemInterface)
