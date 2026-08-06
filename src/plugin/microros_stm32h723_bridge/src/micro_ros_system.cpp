#include "microros_stm32h723_bridge/micro_ros_system.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "yaml-cpp/yaml.h"

namespace microros_stm32h723_bridge
{
namespace
{

constexpr std::array<const char *, MicroRosSystem::kJointCount> kProtocolJointNames = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
  "gripper_finger_left_joint"};

double parameter_or(
  const hardware_interface::HardwareInfo & info, const std::string & key, double fallback)
{
  const auto item = info.hardware_parameters.find(key);
  return item == info.hardware_parameters.end() ? fallback : std::stod(item->second);
}

std::string parameter_or(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto item = info.hardware_parameters.find(key);
  return item == info.hardware_parameters.end() ? fallback : item->second;
}

}  // namespace

MicroRosSystem::~MicroRosSystem()
{
  stop_executor();
}

hardware_interface::CallbackReturn MicroRosSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("MicroRosSystem"), "Expected 7 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    joint_names_.reserve(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto & joint = info_.joints[i];
      if (joint.name != kProtocolJointNames[i]) {
        throw std::runtime_error(
                "joint order must match the fixed STM32 protocol; expected " +
                std::string(kProtocolJointNames[i]) + " at index " + std::to_string(i));
      }
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::runtime_error("each joint must export one position command interface");
      }
      if (joint.state_interfaces.size() != 3) {
        throw std::runtime_error("each joint must export position, velocity and effort state interfaces");
      }
      const std::array<std::string, 3> expected_states = {
        hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT};
      for (std::size_t state_index = 0; state_index < expected_states.size(); ++state_index) {
        if (joint.state_interfaces[state_index].name != expected_states[state_index]) {
          throw std::runtime_error("state interface order must be position, velocity, effort");
        }
      }
      joint_names_.push_back(joint.name);
      lower_limits_[i] = std::stod(joint.command_interfaces[0].min);
      upper_limits_[i] = std::stod(joint.command_interfaces[0].max);
    }

    command_topic_ = parameter_or(info_, "command_topic", command_topic_);
    state_topic_ = parameter_or(info_, "state_topic", state_topic_);
    status_topic_ = parameter_or(info_, "status_topic", status_topic_);
    state_timeout_ = std::chrono::milliseconds(
      static_cast<int64_t>(parameter_or(info_, "state_timeout_ms", 100.0)));
    activation_timeout_ = std::chrono::milliseconds(
      static_cast<int64_t>(parameter_or(info_, "activation_timeout_ms", 3000.0)));
    return_duration_ = std::chrono::milliseconds(
      static_cast<int64_t>(parameter_or(info_, "return_duration_ms", 3000.0)));
    required_joint_mask_ = static_cast<uint32_t>(
      std::stoul(parameter_or(info_, "required_joint_mask", "0x7f"), nullptr, 0));
    zero_tolerance_ = parameter_or(info_, "zero_tolerance", 0.03);

    if (!load_gains(parameter_or(info_, "gains_file", ""))) {
      throw std::runtime_error("failed to load MIT gains");
    }
    const auto effort_node = YAML::LoadFile(parameter_or(info_, "gains_file", ""))["effort_limits"];
    if (!effort_node || effort_node.size() != kJointCount) {
      throw std::runtime_error("effort_limits must contain 7 values");
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      effort_limits_[i] = effort_node[i].as<double>();
    }

    gravity_.initialize(
      parameter_or(info_, "model_urdf_path", ""), joint_names_,
      parameter_or(info_, "joint4_gravity_scale", 0.85));

    node_ = std::make_shared<rclcpp::Node>("microros_stm32h723_hardware");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_publisher_ = node_->create_publisher<msg::JointCommand>(command_topic_, qos);
    state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, qos, std::bind(&MicroRosSystem::joint_state_callback, this, std::placeholders::_1));
    status_subscription_ = node_->create_subscription<msg::HardwareStatus>(
      status_topic_, qos, std::bind(&MicroRosSystem::status_callback, this, std::placeholders::_1));
    safe_stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/safe_stop",
      std::bind(&MicroRosSystem::request_safe_stop, this, std::placeholders::_1, std::placeholders::_2));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("MicroRosSystem"), "Initialization failed: %s", error.what());
    stop_executor();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroRosSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 3);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &state_position_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MicroRosSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &command_position_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn MicroRosSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::unique_lock<std::mutex> lock(feedback_mutex_);
  if (!feedback_cv_.wait_for(
      lock, activation_timeout_,
      [this]() {return feedback_.valid && required_joints_active_.load();}))
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for complete state and required joint mask");
    return hardware_interface::CallbackReturn::ERROR;
  }
  state_position_ = feedback_.position;
  state_velocity_ = feedback_.velocity;
  state_effort_ = feedback_.effort;
  command_position_ = state_position_;
  command_velocity_.fill(0.0);
  command_mode_.store(msg::JointCommand::MODE_HOLD);
  active_.store(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MicroRosSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (disable_sequence_ == 0 || status_.mode != msg::HardwareStatus::MODE_DISABLE ||
    status_.last_command_sequence != disable_sequence_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Refusing deactivate before STM32 confirms DISABLE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroRosSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  Feedback snapshot;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    snapshot = feedback_;
  }
  if (!snapshot.valid || std::chrono::steady_clock::now() - snapshot.received_at > state_timeout_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "STM32 JointState timeout");
    return hardware_interface::return_type::ERROR;
  }
  state_position_ = snapshot.position;
  state_velocity_ = snapshot.velocity;
  state_effort_ = snapshot.effort;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MicroRosSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return hardware_interface::return_type::OK;
  }

  auto targets = command_position_;
  if (!all_finite(targets)) {
    RCLCPP_ERROR(node_->get_logger(), "Rejected non-finite position command");
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    targets[i] = std::clamp(targets[i], lower_limits_[i], upper_limits_[i]);
  }

  if (safe_stop_requested_.load()) {
    if (command_mode_.load() != msg::JointCommand::MODE_RETURN) {
      return_start_ = state_position_;
      return_started_at_ = std::chrono::steady_clock::now();
      command_mode_.store(msg::JointCommand::MODE_RETURN);
    }
    const auto elapsed = std::chrono::steady_clock::now() - return_started_at_;
    const double alpha = std::clamp(
      std::chrono::duration<double>(elapsed).count() /
      std::chrono::duration<double>(return_duration_).count(), 0.0, 1.0);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      targets[i] = return_start_[i] * (1.0 - alpha);
    }
    const bool at_zero = std::all_of(
      state_position_.begin(), state_position_.end(),
      [this](double value) {return std::abs(value) <= zero_tolerance_;});
    if (alpha >= 1.0 && at_zero) {
      command_mode_.store(msg::JointCommand::MODE_DISABLE);
    }
  } else if (command_mode_.load() == msg::JointCommand::MODE_HOLD && targets != state_position_) {
    command_mode_.store(msg::JointCommand::MODE_TRACK);
  }

  std::vector<double> positions(state_position_.begin(), state_position_.end());
  std::vector<double> gravity_effort;
  try {
    gravity_effort = gravity_.compute(positions);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node_->get_logger(), "Gravity computation failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }

  msg::JointCommand message;
  message.header.stamp = node_->now();
  message.mode = command_mode_.load();
  if (message.mode == msg::JointCommand::MODE_DISABLE) {
    if (disable_sequence_ == 0) {
      disable_sequence_ = ++sequence_;
    }
    message.sequence = disable_sequence_;
  } else {
    message.sequence = ++sequence_;
  }
  const auto & gains = selected_gains();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    message.position[i] = static_cast<float>(targets[i]);
    message.velocity[i] = static_cast<float>(command_velocity_[i]);
    message.kp[i] = static_cast<float>(gains.kp[i]);
    message.kd[i] = static_cast<float>(gains.kd[i]);
    const double effort = message.mode == msg::JointCommand::MODE_DISABLE ? 0.0 :
      std::clamp(gravity_effort[i], -effort_limits_[i], effort_limits_[i]);
    message.effort[i] = static_cast<float>(effort);
  }
  command_publisher_->publish(message);
  return hardware_interface::return_type::OK;
}

void MicroRosSystem::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != kJointCount || message->position.size() != kJointCount ||
    message->velocity.size() != kJointCount || message->effort.size() != kJointCount)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Ignoring incomplete JointState");
    return;
  }
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < message->name.size(); ++i) {
    indices.emplace(message->name[i], i);
  }
  Feedback next;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto item = indices.find(joint_names_[i]);
    if (item == indices.end()) {
      return;
    }
    next.position[i] = message->position[item->second];
    next.velocity[i] = message->velocity[item->second];
    next.effort[i] = message->effort[item->second];
  }
  if (!all_finite(next.position) || !all_finite(next.velocity) || !all_finite(next.effort)) {
    return;
  }
  next.received_at = std::chrono::steady_clock::now();
  next.valid = true;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_ = next;
  }
  feedback_cv_.notify_all();
}

void MicroRosSystem::status_callback(const msg::HardwareStatus::SharedPtr message)
{
  const bool required_active =
    (message->active_joint_mask & required_joint_mask_) == required_joint_mask_;
  required_joints_active_.store(required_active);
  if (!required_active) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Required STM32 joints are inactive");
  }
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = *message;
  feedback_cv_.notify_all();
}

void MicroRosSystem::request_safe_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!active_.load()) {
    response->success = false;
    response->message = "hardware is not active";
    return;
  }
  safe_stop_requested_.store(true);
  response->success = true;
  response->message = "safe return requested; monitor /stm32/status for DISABLE confirmation";
}

void MicroRosSystem::stop_executor()
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  executor_.reset();
  node_.reset();
}

bool MicroRosSystem::load_gains(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  const YAML::Node root = YAML::LoadFile(path);
  const auto load_group = [&root](const std::string & name, Gains & target) {
      const auto group = root[name];
      if (!group || !group["kp"] || !group["kd"] ||
        group["kp"].size() != kJointCount || group["kd"].size() != kJointCount)
      {
        throw std::runtime_error("invalid gains group: " + name);
      }
      for (std::size_t i = 0; i < kJointCount; ++i) {
        target.kp[i] = group["kp"][i].as<double>();
        target.kd[i] = group["kd"][i].as<double>();
      }
    };
  load_group("hold", hold_gains_);
  load_group("track", track_gains_);
  load_group("return", return_gains_);
  load_group("idle", idle_gains_);
  return true;
}

bool MicroRosSystem::all_finite(const std::array<double, kJointCount> & values) const
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

const MicroRosSystem::Gains & MicroRosSystem::selected_gains() const
{
  switch (command_mode_.load()) {
    case msg::JointCommand::MODE_TRACK:
      return track_gains_;
    case msg::JointCommand::MODE_RETURN:
      return return_gains_;
    case msg::JointCommand::MODE_DISABLE:
      return idle_gains_;
    default:
      return hold_gains_;
  }
}

}  // namespace microros_stm32h723_bridge
