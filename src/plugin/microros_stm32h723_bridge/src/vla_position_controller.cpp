#include "microros_stm32h723_bridge/vla_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace microros_stm32h723_bridge
{

controller_interface::CallbackReturn VlaPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    auto_declare<std::string>("chunk_topic", "/vla/joint_chunk");
    auto_declare<double>("command_timeout_sec", 0.2);
    auto_declare<double>("max_delta_per_sec", 1.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  chunk_topic_ = get_node()->get_parameter("chunk_topic").as_string();
  command_timeout_sec_ = get_node()->get_parameter("command_timeout_sec").as_double();
  max_delta_per_sec_ = get_node()->get_parameter("max_delta_per_sec").as_double();

  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints must be a non-empty list");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (chunk_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "chunk_topic must be non-empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(command_timeout_sec_ > 0.0) || !std::isfinite(command_timeout_sec_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_timeout_sec must be finite and > 0");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(max_delta_per_sec_ > 0.0) || !std::isfinite(max_delta_per_sec_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "max_delta_per_sec must be finite and > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  pending_target_.assign(joints_.size(), 0.0);
  last_command_.assign(joints_.size(), 0.0);
  active_timeout_sec_ = command_timeout_sec_;
  has_valid_command_ = false;
  hold_warned_ = false;

  chunk_subscription_ =
    get_node()->create_subscription<microros_stm32h723_bridge::msg::VlaJointChunk>(
      chunk_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&VlaPositionController::chunk_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "VlaPositionController configured: topic=%s timeout=%.3f s max_delta=%.3f rad/s joints=%zu",
    chunk_topic_.c_str(), command_timeout_sec_, max_delta_per_sec_, joints_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
VlaPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
VlaPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::CallbackReturn VlaPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double position = state_interfaces_[i].get_value();
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Non-finite position state on activate for %s",
        joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    last_command_[i] = position;
    pending_target_[i] = position;
    command_interfaces_[i].set_value(position);
  }
  last_chunk_time_ = get_node()->now();
  active_timeout_sec_ = command_timeout_sec_;
  has_valid_command_ = true;
  hold_warned_ = false;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "VlaPositionController active; holding current joint positions until a chunk arrives");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  has_valid_command_ = false;
  for (std::size_t i = 0; i < command_interfaces_.size() && i < last_command_.size(); ++i) {
    command_interfaces_[i].set_value(last_command_[i]);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void VlaPositionController::chunk_callback(
  const microros_stm32h723_bridge::msg::VlaJointChunk::SharedPtr message)
{
  if (!message) {
    return;
  }
  if (message->positions.size() != joints_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Ignoring VlaJointChunk: expected %zu positions, got %zu",
      joints_.size(), message->positions.size());
    return;
  }
  for (std::size_t i = 0; i < message->positions.size(); ++i) {
    if (!std::isfinite(message->positions[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring VlaJointChunk: non-finite position at index %zu", i);
      return;
    }
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    pending_target_[i] = static_cast<double>(message->positions[i]);
  }
  last_chunk_time_ = get_node()->now();
  if (std::isfinite(message->timeout_sec) && message->timeout_sec > 0.0) {
    active_timeout_sec_ = message->timeout_sec;
  } else {
    active_timeout_sec_ = command_timeout_sec_;
  }
  has_valid_command_ = true;
  hold_warned_ = false;
}

controller_interface::return_type VlaPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = std::clamp(period.seconds(), 0.0, 0.1);
  const double max_step = max_delta_per_sec_ * dt;

  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!has_valid_command_ || last_command_.size() != joints_.size()) {
    return controller_interface::return_type::ERROR;
  }

  const bool stale =
    (time - last_chunk_time_).seconds() > active_timeout_sec_;
  if (stale) {
    // Hold last commanded positions. Upper layer / firmware mode should be HOLD.
    if (!hold_warned_) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "VlaJointChunk timed out (%.3f s); holding last positions. "
        "Upper layer should keep MODE_HOLD while inference is stale.",
        active_timeout_sec_);
      hold_warned_ = true;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      command_interfaces_[i].set_value(last_command_[i]);
    }
    return controller_interface::return_type::OK;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    double delta = pending_target_[i] - last_command_[i];
    if (max_step > 0.0) {
      delta = std::clamp(delta, -max_step, max_step);
    }
    last_command_[i] += delta;
    if (!std::isfinite(last_command_[i])) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Non-finite command for %s; freezing previous value", joints_[i].c_str());
      return controller_interface::return_type::ERROR;
    }
    command_interfaces_[i].set_value(last_command_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace microros_stm32h723_bridge

PLUGINLIB_EXPORT_CLASS(
  microros_stm32h723_bridge::VlaPositionController,
  controller_interface::ControllerInterface)
