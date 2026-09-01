#include "microros_stm32h723_bridge/gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace microros_stm32h723_bridge
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    auto_declare<std::string>("model_urdf_path", "");
    auto_declare<std::vector<double>>("gravity_scales", std::vector<double>(6, 1.0));
    auto_declare<std::vector<double>>(
      "effort_limits", {5.0, 5.0, 5.0, 3.0, 3.0, 3.0});
    auto_declare<double>("startup_hold_duration_sec", 5.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  model_urdf_path_ = get_node()->get_parameter("model_urdf_path").as_string();
  gravity_scales_ = get_node()->get_parameter("gravity_scales").as_double_array();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  startup_hold_duration_sec_ =
    get_node()->get_parameter("startup_hold_duration_sec").as_double();

  if (joints_.empty() || gravity_scales_.size() != joints_.size() ||
    effort_limits_.size() != joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "joints, gravity_scales and effort_limits must have the same non-zero length");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(startup_hold_duration_sec_) || startup_hold_duration_sec_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "startup_hold_duration_sec must be non-negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (model_urdf_path_.empty()) {
    model_urdf_path_ =
      ament_index_cpp::get_package_share_directory("rebotarm_gazebo") + "/urdf/rebotarm.urdf";
  }
  try {
    gravity_.initialize(model_urdf_path_, joints_, 1.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Gravity model initialization failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "Using gravity model: %s", model_urdf_path_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State &)
{
  startup_positions_.resize(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    startup_positions_[i] = state_interfaces_[i].get_value();
    if (!std::isfinite(startup_positions_[i])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot capture finite startup joint positions");
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[2 * i].set_value(startup_positions_[i]);
    command_interfaces_[2 * i + 1].set_value(0.0);
  }
  activated_at_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_node()->get_logger(), "Holding startup position for %.2f seconds",
    startup_hold_duration_sec_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      command_interface.set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<double> positions(joints_.size(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    positions[i] = state_interfaces_[i].get_value();
    if (!std::isfinite(positions[i])) {
      return controller_interface::return_type::ERROR;
    }
  }

  std::vector<double> gravity_effort;
  try {
    gravity_effort = gravity_.compute(positions);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Gravity computation failed: %s", error.what());
    return controller_interface::return_type::ERROR;
  }

  const double active_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - activated_at_).count();
  const bool hold_startup_position = active_seconds < startup_hold_duration_sec_;

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double effort = std::clamp(
      gravity_effort[i] * gravity_scales_[i], -effort_limits_[i], effort_limits_[i]);
    command_interfaces_[2 * i].set_value(
      hold_startup_position ? startup_positions_[i] : positions[i]);
    command_interfaces_[2 * i + 1].set_value(effort);
  }
  return controller_interface::return_type::OK;
}

}  // namespace microros_stm32h723_bridge

PLUGINLIB_EXPORT_CLASS(
  microros_stm32h723_bridge::GravityCompensationController,
  controller_interface::ControllerInterface)
