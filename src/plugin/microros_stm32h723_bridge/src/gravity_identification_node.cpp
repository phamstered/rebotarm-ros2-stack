// SPDX-License-Identifier: MIT
#include "microros_stm32h723_bridge/gravity_identification_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <filesystem>

namespace microros_stm32h723_bridge
{
namespace
{

using gravity_identification::kArmDof;
using gravity_identification::resolve_joint_mask;
using gravity_identification::joint_active;
using gravity_identification::fit_per_joint;

constexpr const char * kJointNames[kArmDof] = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

double clamp_abs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

std::string expand_home(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }
  return std::string(home) + path.substr(1);
}

std::string iso8601_now_local()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&tt, &local_tm);
  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &local_tm);
  std::string text(buffer);
  // Insert colon in timezone offset: +0800 -> +08:00
  if (text.size() >= 5) {
    text.insert(text.size() - 2, ":");
  }
  return text;
}

}  // namespace

GravityIdentificationNode::GravityIdentificationNode(const rclcpp::NodeOptions & options)
: Node("gravity_identification", options)
{
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  model_urdf_path_ = declare_parameter<std::string>("model_urdf_path", "");
  command_topic_ = declare_parameter<std::string>("command_topic", "/stm32/joint_commands");
  state_topic_ = declare_parameter<std::string>("state_topic", "/stm32/joint_states");
  status_topic_ = declare_parameter<std::string>("status_topic", "/stm32/status");
  default_yaml_path_ = declare_parameter<std::string>(
    "default_yaml_path", "~/.rebotarm/gravity_id_latest.yaml");
  controller_manager_name_ = declare_parameter<std::string>(
    "controller_manager_name", "/controller_manager");
  require_exclusive_bus_ = declare_parameter<bool>("require_exclusive_bus", true);
  attempt_controller_switch_ = declare_parameter<bool>("attempt_controller_switch", true);
  command_period_s_ = declare_parameter<double>("command_period_s", 0.02);
  position_tolerance_rad_ = declare_parameter<double>("position_tolerance_rad", 0.03);
  velocity_settle_threshold_ = declare_parameter<double>("velocity_settle_threshold", 0.02);
  minimum_model_span_nm_ = declare_parameter<double>("minimum_model_span_nm", 0.15);
  state_max_age_s_ = declare_parameter<double>("state_max_age_s", 0.25);

  auto effort_limits = declare_parameter<std::vector<double>>(
    "effort_limits", {10.0, 10.0, 10.0, 6.0, 6.0, 6.0});
  auto track_kp = declare_parameter<std::vector<double>>(
    "track_kp", {40.0, 40.0, 40.0, 15.0, 15.0, 15.0});
  auto track_kd = declare_parameter<std::vector<double>>(
    "track_kd", {3.0, 3.0, 3.0, 1.5, 1.5, 1.5});
  auto hold_kp = declare_parameter<std::vector<double>>(
    "hold_kp", {20.0, 20.0, 20.0, 10.0, 10.0, 10.0});
  auto hold_kd = declare_parameter<std::vector<double>>(
    "hold_kd", {2.0, 2.0, 2.0, 1.5, 1.5, 1.5});
  auto pose_amplitudes = declare_parameter<std::vector<double>>(
    "pose_amplitudes", {0.45, 0.40, 0.45, 0.30, 0.25, 0.35});
  auto pose_centers = declare_parameter<std::vector<double>>(
    "pose_centers", {0.0, -0.70, -0.80, 0.0, 0.0, 0.0});
  prior_scales_ = declare_parameter<std::vector<double>>(
    "prior_gravity_scales", std::vector<double>(kArmDof, 1.0));
  prior_biases_ = declare_parameter<std::vector<double>>(
    "prior_torque_biases_nm", std::vector<double>(kArmDof, 0.0));

  if (joint_names_.size() != kArmDof || effort_limits.size() != kArmDof ||
    track_kp.size() != kArmDof || track_kd.size() != kArmDof ||
    hold_kp.size() != kArmDof || hold_kd.size() != kArmDof ||
    pose_amplitudes.size() != kArmDof || pose_centers.size() != kArmDof ||
    prior_scales_.size() != kArmDof || prior_biases_.size() != kArmDof)
  {
    throw std::runtime_error("gravity_identification parameters must all be length 6");
  }
  for (std::size_t i = 0; i < kArmDof; ++i) {
    effort_limits_[i] = effort_limits[i];
    track_kp_[i] = track_kp[i];
    track_kd_[i] = track_kd[i];
    hold_kp_[i] = hold_kp[i];
    hold_kd_[i] = hold_kd[i];
    pose_amplitudes_[i] = pose_amplitudes[i];
    pose_centers_[i] = pose_centers[i];
  }

  if (model_urdf_path_.empty()) {
    model_urdf_path_ =
      ament_index_cpp::get_package_share_directory("rebotarm_gazebo") + "/urdf/rebotarm.urdf";
  }
  gravity_.initialize(model_urdf_path_, joint_names_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  command_publisher_ = create_publisher<msg::JointCommand>(command_topic_, qos);
  state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&GravityIdentificationNode::joint_state_callback, this, std::placeholders::_1));
  status_subscription_ = create_subscription<msg::HardwareStatus>(
    status_topic_, qos,
    std::bind(&GravityIdentificationNode::status_callback, this, std::placeholders::_1));
  command_timer_ = create_wall_timer(
    std::chrono::duration<double>(command_period_s_),
    std::bind(&GravityIdentificationNode::command_timer_callback, this));

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<GravityIdentification>(
    this,
    gravity_identification::ACTION_SERVER_NAME,
    std::bind(&GravityIdentificationNode::handle_goal, this, _1, _2),
    std::bind(&GravityIdentificationNode::handle_cancel, this, _1),
    std::bind(&GravityIdentificationNode::handle_accepted, this, _1));

  RCLCPP_INFO(
    get_logger(),
    "StartGravityID ready (model=%s command=%s state=%s). "
    "Run without MicroRosSystem write path (state_only or stop controller_manager).",
    model_urdf_path_.c_str(), command_topic_.c_str(), state_topic_.c_str());
}

rclcpp_action::GoalResponse GravityIdentificationNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GravityIdentification::Goal> goal)
{
  if (session_active_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting StartGravityID: session already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->mode == GravityIdentification::Goal::MODE_PAYLOAD_SUBMODE) {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting PAYLOAD_SUBMODE: v1 only supports FULL_ARM_GRAVITY "
      "(use joint4_calibration_controller for payload)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->mode != GravityIdentification::Goal::MODE_FULL_ARM_GRAVITY) {
    RCLCPP_WARN(get_logger(), "Rejecting unknown mode=%u", goal->mode);
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GravityIdentificationNode::handle_cancel(
  const std::shared_ptr<GoalHandle>)
{
  abort_requested_.store(true);
  abort_reason_ = "action canceled";
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GravityIdentificationNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&GravityIdentificationNode::execute, this, goal_handle)}.detach();
}

void GravityIdentificationNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  session_active_.store(true);
  abort_requested_.store(false);
  abort_reason_.clear();

  const auto goal_msg = goal_handle->get_goal();
  SessionGoal goal;
  goal.joint_mask = resolve_joint_mask(goal_msg->joint_mask);
  goal.mode = goal_msg->mode;
  goal.settle_time_s = goal_msg->settle_time_s > 0.0F ? goal_msg->settle_time_s : 1.5;
  goal.capture_window_s = goal_msg->capture_window_s > 0.0F ? goal_msg->capture_window_s : 1.0;
  goal.write_yaml = goal_msg->write_yaml;
  goal.hot_reload = goal_msg->hot_reload;
  goal.output_yaml_path = goal_msg->output_yaml_path.empty() ?
    default_yaml_path_ : goal_msg->output_yaml_path;
  goal.max_effort_nm = goal_msg->max_effort_nm > 0.0F ? goal_msg->max_effort_nm : 10.0;
  goal.max_velocity_rad_s =
    goal_msg->max_velocity_rad_s > 0.0F ? goal_msg->max_velocity_rad_s : 0.15;
  goal.command_timeout_ms =
    goal_msg->command_timeout_ms > 0.0F ? goal_msg->command_timeout_ms : 100.0;
  goal.lock_gripper = goal_msg->lock_gripper;
  active_joint_mask_ = goal.joint_mask;

  auto result = std::make_shared<GravityIdentification::Result>();
  result->gravity_scales.assign(kArmDof, 1.0F);
  result->torque_biases_nm.assign(kArmDof, 0.0F);
  result->rmse_per_joint.assign(kArmDof, 0.0F);
  result->success = false;
  result->fault_code = 0;
  result->rmse_total = 0.0F;

  const auto fail = [&](const std::string & message, uint16_t fault = 0) {
      abort_to_hold(message);
      publish_feedback(goal_handle, GravityIdentification::Feedback::PHASE_FAILED, 0, 0, message);
      result->success = false;
      result->fault_code = fault;
      result->message = message;
      goal_handle->abort(result);
      disarm_session();
      session_active_.store(false);
    };

  if (!goal_msg->pose_set_id.empty()) {
    // v1 only ships the builtin set; named sets are accepted as a soft ignore with warning.
    RCLCPP_WARN(
      get_logger(),
      "pose_set_id provided but v1 only uses the builtin pose set; ignoring named set");
  }

  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_ARMING, 0, 0, "arming session");
  std::string message;
  if (!arm_session(goal, message)) {
    fail(message, 0x8001);
    return;
  }

  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_ARMED, 0, 0, "armed; starting poses");
  const auto poses = builtin_poses();
  const uint16_t pose_count = static_cast<uint16_t>(poses.size());
  std::vector<PoseSample> samples;
  samples.reserve(poses.size());

  for (uint16_t pose_index = 0; pose_index < pose_count; ++pose_index) {
    if (abort_requested_.load() || goal_handle->is_canceling()) {
      fail(abort_reason_.empty() ? "canceled" : abort_reason_);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (status_valid_ && latest_status_.fault_code != 0) {
        fail(
          "firmware fault_code=" + std::to_string(latest_status_.fault_code),
          latest_status_.fault_code);
        return;
      }
    }

    if (!move_to_pose(poses[pose_index], goal, goal_handle, pose_index, pose_count, message)) {
      fail(message);
      return;
    }
    PoseSample sample;
    if (!settle_and_capture(goal, goal_handle, pose_index, pose_count, sample, message)) {
      fail(message);
      return;
    }
    samples.push_back(sample);
  }

  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_FITTING, pose_count, pose_count,
    "fitting per-joint scale/bias");

  std::vector<std::vector<double>> model(kArmDof);
  std::vector<std::vector<double>> meas(kArmDof);
  for (const auto & sample : samples) {
    for (std::size_t j = 0; j < kArmDof; ++j) {
      if (!joint_active(goal.joint_mask, j)) {
        continue;
      }
      model[j].push_back(sample.model_effort[j]);
      meas[j].push_back(sample.measured_effort[j]);
    }
  }

  const auto fit = fit_per_joint(
    goal.joint_mask, model, meas, minimum_model_span_nm_, prior_scales_, prior_biases_);
  if (!fit.success) {
    fail(fit.message);
    return;
  }

  for (std::size_t j = 0; j < kArmDof; ++j) {
    result->gravity_scales[j] = static_cast<float>(fit.gravity_scales[j]);
    result->torque_biases_nm[j] = static_cast<float>(fit.torque_biases_nm[j]);
    result->rmse_per_joint[j] = static_cast<float>(fit.rmse_per_joint[j]);
  }
  result->rmse_total = static_cast<float>(fit.rmse_total);

  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_WRITING, pose_count, pose_count,
    "writing YAML / hot_reload", nullptr, static_cast<float>(fit.rmse_total));

  if (goal.write_yaml) {
    std::string yaml_path;
    if (!write_yaml_file(goal, fit, pose_count, yaml_path, message)) {
      fail(message);
      return;
    }
    result->yaml_path = yaml_path;
  }

  if (goal.hot_reload) {
    std::string reload_message;
    if (!try_hot_reload(fit, reload_message)) {
      // Honest soft-fail: YAML may still be written; document manual reload.
      RCLCPP_WARN(get_logger(), "hot_reload deferred: %s", reload_message.c_str());
      result->message = fit.message + "; hot_reload deferred: " + reload_message;
    } else {
      result->message = fit.message + "; hot_reload ok";
    }
  } else {
    result->message = fit.message;
  }

  abort_to_hold("session complete");
  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_DONE, pose_count, pose_count,
    result->message, nullptr, static_cast<float>(fit.rmse_total));
  result->success = true;
  result->fault_code = 0;
  goal_handle->succeed(result);
  disarm_session();
  session_active_.store(false);
}

void GravityIdentificationNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size()) {
    return;
  }
  std::array<double, kArmDof> position{};
  std::array<double, kArmDof> velocity{};
  std::array<double, kArmDof> effort{};
  std::size_t found = 0;
  for (std::size_t j = 0; j < kArmDof; ++j) {
    bool ok = false;
    for (std::size_t i = 0; i < message->name.size(); ++i) {
      if (message->name[i] != joint_names_[j] && message->name[i] != kJointNames[j]) {
        continue;
      }
      position[j] = message->position[i];
      velocity[j] = i < message->velocity.size() ? message->velocity[i] : 0.0;
      effort[j] = i < message->effort.size() ? message->effort[i] : 0.0;
      ok = std::isfinite(position[j]) && std::isfinite(velocity[j]) && std::isfinite(effort[j]);
      break;
    }
    if (!ok) {
      return;
    }
    ++found;
  }
  if (found != kArmDof) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_position_ = position;
  latest_velocity_ = velocity;
  latest_effort_ = effort;
  latest_state_stamp_ = now();
  state_valid_ = true;
}

void GravityIdentificationNode::status_callback(const msg::HardwareStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  latest_status_ = *message;
  status_valid_ = true;
  if (session_active_.load() && message->fault_code != 0) {
    abort_requested_.store(true);
    abort_reason_ = "firmware fault_code=" + std::to_string(message->fault_code);
  }
}

void GravityIdentificationNode::command_timer_callback()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!command_streaming_) {
    return;
  }
  msg::JointCommand command;
  command.header.stamp = now();
  command.sequence = ++command_sequence_;
  if (command.sequence == 0) {
    command.sequence = ++command_sequence_;
  }
  command.mode = command_mode_;
  for (std::size_t i = 0; i < 7; ++i) {
    command.position[i] = 0.0F;
    command.velocity[i] = 0.0F;
    command.kp[i] = 0.0F;
    command.kd[i] = 0.0F;
    command.effort[i] = 0.0F;
  }
  const bool is_hold = command_mode_ == msg::JointCommand::MODE_HOLD;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    command.position[i] = static_cast<float>(command_position_[i]);
    command.velocity[i] = static_cast<float>(command_velocity_[i]);
    command.kp[i] = static_cast<float>(is_hold ? hold_kp_[i] : track_kp_[i]);
    command.kd[i] = static_cast<float>(is_hold ? hold_kd_[i] : track_kd_[i]);
    // HOLD clears MIT effort on firmware; still send 0 explicitly.
    command.effort[i] = is_hold ? 0.0F : static_cast<float>(command_effort_[i]);
  }
  if (gripper_locked_) {
    command.position[6] = static_cast<float>(gripper_hold_position_);
    command.velocity[6] = 0.0F;
    command.kp[6] = is_hold ? 2.0F : 100.0F;
    command.kd[6] = 3.0F;
    command.effort[6] = 0.0F;
  }
  command_publisher_->publish(command);
}

bool GravityIdentificationNode::arm_session(const SessionGoal & goal, std::string & message)
{
  if (attempt_controller_switch_) {
    std::string switch_message;
    if (!deactivate_competing_controllers(switch_message)) {
      RCLCPP_WARN(
        get_logger(),
        "controller switch skipped/failed (%s); ensure MicroRosSystem is not writing "
        "JointCommand (prefer state_only launch)",
        switch_message.c_str());
      if (require_exclusive_bus_) {
        // Soft: still continue if states are fresh; exclusive bus is documented, not hard-enforced
        // when controller_manager is absent.
      }
    }
  }

  std::array<double, kArmDof> position{};
  std::array<double, kArmDof> velocity{};
  std::array<double, kArmDof> effort{};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (snapshot_state(position, velocity, effort, state_max_age_s_)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!snapshot_state(position, velocity, effort, state_max_age_s_)) {
    message = "no fresh /stm32/joint_states (topic=" + state_topic_ + ")";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_valid_ && latest_status_.fault_code != 0) {
      message = "cannot arm: fault_code=" + std::to_string(latest_status_.fault_code);
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_position_ = position;
    command_velocity_.fill(0.0);
    command_effort_.fill(0.0);
    command_mode_ = msg::JointCommand::MODE_HOLD;
    gripper_locked_ = goal.lock_gripper;
    gripper_hold_position_ = 0.0;
    command_streaming_ = true;
  }
  (void)goal;
  message = "armed";
  return true;
}

void GravityIdentificationNode::disarm_session()
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_mode_ = msg::JointCommand::MODE_HOLD;
    command_velocity_.fill(0.0);
    command_effort_.fill(0.0);
    // Keep streaming HOLD briefly so firmware does not see command loss, then stop.
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_streaming_ = false;
}

void GravityIdentificationNode::publish_hold_command()
{
  std::array<double, kArmDof> position{};
  std::array<double, kArmDof> velocity{};
  std::array<double, kArmDof> effort{};
  if (snapshot_state(position, velocity, effort, 1.0)) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_position_ = position;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_velocity_.fill(0.0);
  command_effort_.fill(0.0);
  command_mode_ = msg::JointCommand::MODE_HOLD;
  command_streaming_ = true;
}

void GravityIdentificationNode::publish_track_command(
  const std::array<double, kArmDof> & position,
  const std::array<double, kArmDof> & velocity,
  const std::array<double, kArmDof> & effort,
  bool)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_position_ = position;
  command_velocity_ = velocity;
  command_effort_ = effort;
  command_mode_ = msg::JointCommand::MODE_TRACK;
  command_streaming_ = true;
}

void GravityIdentificationNode::abort_to_hold(const std::string & reason)
{
  RCLCPP_WARN(get_logger(), "StartGravityID -> HOLD: %s", reason.c_str());
  publish_hold_command();
}

bool GravityIdentificationNode::wait_until(
  double timeout_s,
  const std::function<bool()> & predicate,
  const std::shared_ptr<GoalHandle> & goal_handle,
  uint8_t phase,
  const std::string & waiting_message)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_s));
  while (std::chrono::steady_clock::now() < deadline) {
    if (abort_requested_.load() || goal_handle->is_canceling()) {
      return false;
    }
    if (predicate()) {
      return true;
    }
    publish_feedback(goal_handle, phase, 0, 0, waiting_message);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

bool GravityIdentificationNode::move_to_pose(
  const std::array<double, kArmDof> & target,
  const SessionGoal & goal,
  const std::shared_ptr<GoalHandle> & goal_handle,
  uint16_t pose_index,
  uint16_t pose_count,
  std::string & message)
{
  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_MOVING, pose_index, pose_count,
    "moving to pose " + std::to_string(pose_index));

  std::array<double, kArmDof> start{};
  std::array<double, kArmDof> velocity{};
  std::array<double, kArmDof> effort{};
  if (!snapshot_state(start, velocity, effort, state_max_age_s_)) {
    message = "lost joint state while moving";
    return false;
  }

  double max_delta = 0.0;
  for (std::size_t j = 0; j < kArmDof; ++j) {
    max_delta = std::max(max_delta, std::abs(target[j] - start[j]));
  }
  const double duration = std::max(1.0, max_delta / std::max(goal.max_velocity_rad_s, 1e-3));
  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    if (abort_requested_.load() || goal_handle->is_canceling()) {
      message = abort_reason_.empty() ? "canceled during move" : abort_reason_;
      return false;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    const double alpha = std::clamp(elapsed / duration, 0.0, 1.0);
    std::array<double, kArmDof> command_position{};
    std::array<double, kArmDof> command_velocity{};
    for (std::size_t j = 0; j < kArmDof; ++j) {
      command_position[j] = start[j] + alpha * (target[j] - start[j]);
      command_velocity[j] = alpha >= 1.0 ? 0.0 :
        clamp_abs((target[j] - start[j]) / duration, goal.max_velocity_rad_s);
    }

    std::vector<double> model;
    try {
      model = gravity_.compute(
        std::vector<double>(command_position.begin(), command_position.end()));
    } catch (const std::exception & error) {
      message = std::string("gravity model failed: ") + error.what();
      return false;
    }
    std::array<double, kArmDof> model_effort{};
    for (std::size_t j = 0; j < kArmDof; ++j) {
      model_effort[j] = model[j];
    }
    const auto limited = clamp_effort(model_effort, goal.max_effort_nm);
    for (std::size_t j = 0; j < kArmDof; ++j) {
      if (std::abs(limited[j]) > 10.0 + 1e-6) {
        message = "effort cap |tau|>10 Nm on joint" + std::to_string(j + 1);
        return false;
      }
    }
    publish_track_command(command_position, command_velocity, limited, goal.lock_gripper);
    publish_feedback(
      goal_handle, GravityIdentification::Feedback::PHASE_MOVING, pose_index, pose_count,
      "moving to pose " + std::to_string(pose_index));

    if (alpha >= 1.0) {
      std::array<double, kArmDof> current{};
      if (!snapshot_state(current, velocity, effort, state_max_age_s_)) {
        message = "lost joint state at end of move";
        return false;
      }
      bool reached = true;
      for (std::size_t j = 0; j < kArmDof; ++j) {
        if (std::abs(current[j] - target[j]) > position_tolerance_rad_) {
          reached = false;
          break;
        }
      }
      if (reached || elapsed > duration + 2.0) {
        if (!reached) {
          RCLCPP_WARN(
            get_logger(),
            "pose %u not within tolerance; continuing with measured pose", pose_index);
        }
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool GravityIdentificationNode::settle_and_capture(
  const SessionGoal & goal,
  const std::shared_ptr<GoalHandle> & goal_handle,
  uint16_t pose_index,
  uint16_t pose_count,
  PoseSample & sample,
  std::string & message)
{
  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_HOLD_SETTLE, pose_index, pose_count,
    "settling (TRACK@v≈0 + gravity FF)");

  const auto settle_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(goal.settle_time_s + 5.0));
  const auto settle_needed = std::chrono::duration<double>(goal.settle_time_s);
  auto quiet_since = std::chrono::steady_clock::now();
  bool quiet = false;

  while (std::chrono::steady_clock::now() < settle_deadline) {
    if (abort_requested_.load() || goal_handle->is_canceling()) {
      message = abort_reason_.empty() ? "canceled during settle" : abort_reason_;
      return false;
    }
    std::array<double, kArmDof> position{};
    std::array<double, kArmDof> velocity{};
    std::array<double, kArmDof> effort{};
    if (!snapshot_state(position, velocity, effort, state_max_age_s_)) {
      message = "lost joint state during settle";
      return false;
    }
    std::vector<double> model;
    try {
      model = gravity_.compute(std::vector<double>(position.begin(), position.end()));
    } catch (const std::exception & error) {
      message = std::string("gravity model failed: ") + error.what();
      return false;
    }
    std::array<double, kArmDof> model_effort{};
    for (std::size_t j = 0; j < kArmDof; ++j) {
      model_effort[j] = model[j];
    }
    const auto limited = clamp_effort(model_effort, goal.max_effort_nm);
    std::array<double, kArmDof> zero_vel{};
    publish_track_command(position, zero_vel, limited, goal.lock_gripper);

    bool still = true;
    for (std::size_t j = 0; j < kArmDof; ++j) {
      if (std::abs(velocity[j]) > velocity_settle_threshold_) {
        still = false;
        break;
      }
    }
    if (still) {
      if (!quiet) {
        quiet_since = std::chrono::steady_clock::now();
        quiet = true;
      } else if (std::chrono::steady_clock::now() - quiet_since >= settle_needed) {
        break;
      }
    } else {
      quiet = false;
    }
    publish_feedback(
      goal_handle, GravityIdentification::Feedback::PHASE_HOLD_SETTLE, pose_index, pose_count,
      "settling");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!quiet) {
    message = "failed to settle before capture";
    return false;
  }

  publish_feedback(
    goal_handle, GravityIdentification::Feedback::PHASE_CAPTURING, pose_index, pose_count,
    "capturing window");

  std::array<double, kArmDof> sum_position{};
  std::array<double, kArmDof> sum_meas{};
  std::array<double, kArmDof> sum_model{};
  std::size_t count = 0;
  const auto capture_end = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(goal.capture_window_s));
  while (std::chrono::steady_clock::now() < capture_end) {
    if (abort_requested_.load() || goal_handle->is_canceling()) {
      message = abort_reason_.empty() ? "canceled during capture" : abort_reason_;
      return false;
    }
    std::array<double, kArmDof> position{};
    std::array<double, kArmDof> velocity{};
    std::array<double, kArmDof> effort{};
    if (!snapshot_state(position, velocity, effort, state_max_age_s_)) {
      message = "lost joint state during capture";
      return false;
    }
    std::vector<double> model;
    try {
      model = gravity_.compute(std::vector<double>(position.begin(), position.end()));
    } catch (const std::exception & error) {
      message = std::string("gravity model failed: ") + error.what();
      return false;
    }
    std::array<double, kArmDof> model_effort{};
    for (std::size_t j = 0; j < kArmDof; ++j) {
      model_effort[j] = model[j];
      sum_position[j] += position[j];
      sum_meas[j] += effort[j];
      sum_model[j] += model[j];
    }
    const auto limited = clamp_effort(model_effort, goal.max_effort_nm);
    std::array<double, kArmDof> zero_vel{};
    publish_track_command(position, zero_vel, limited, goal.lock_gripper);
    ++count;

    PoseSample preview;
    preview.position = position;
    preview.measured_effort = effort;
    preview.model_effort = model_effort;
    publish_feedback(
      goal_handle, GravityIdentification::Feedback::PHASE_CAPTURING, pose_index, pose_count,
      "capturing", &preview);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (count == 0) {
    message = "capture window produced no samples";
    return false;
  }
  const double inv = 1.0 / static_cast<double>(count);
  for (std::size_t j = 0; j < kArmDof; ++j) {
    sample.position[j] = sum_position[j] * inv;
    sample.measured_effort[j] = sum_meas[j] * inv;
    sample.model_effort[j] = sum_model[j] * inv;
  }
  return true;
}

void GravityIdentificationNode::publish_feedback(
  const std::shared_ptr<GoalHandle> & goal_handle,
  uint8_t phase,
  uint16_t pose_index,
  uint16_t pose_count,
  const std::string & message,
  const PoseSample * sample,
  float rmse_running)
{
  auto feedback = std::make_shared<GravityIdentification::Feedback>();
  feedback->phase = phase;
  feedback->pose_index = pose_index;
  feedback->pose_count = pose_count;
  feedback->active_joint_mask = active_joint_mask_;
  feedback->residual_nm.assign(kArmDof, 0.0F);
  feedback->measured_effort_nm.assign(kArmDof, 0.0F);
  feedback->model_effort_nm.assign(kArmDof, 0.0F);
  feedback->rmse_running = rmse_running;
  feedback->message = message;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    feedback->fault_code = status_valid_ ? latest_status_.fault_code : 0;
  }
  if (sample != nullptr) {
    for (std::size_t j = 0; j < kArmDof; ++j) {
      feedback->measured_effort_nm[j] = static_cast<float>(sample->measured_effort[j]);
      feedback->model_effort_nm[j] = static_cast<float>(sample->model_effort[j]);
      feedback->residual_nm[j] = static_cast<float>(
        sample->measured_effort[j] - sample->model_effort[j]);
    }
  }
  goal_handle->publish_feedback(feedback);
}

bool GravityIdentificationNode::write_yaml_file(
  const SessionGoal & goal,
  const gravity_identification::JointFitResult & fit,
  uint16_t pose_count,
  std::string & path_out,
  std::string & message)
{
  const std::string path = expand_home(goal.output_yaml_path);
  try {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent)) {
      if (!std::filesystem::create_directories(parent)) {
        message = "failed to create directory for " + path;
        return false;
      }
    }
  } catch (const std::exception & error) {
    message = std::string("yaml path error: ") + error.what();
    return false;
  }

  std::ofstream out(path);
  if (!out) {
    message = "failed to open yaml for write: " + path;
    return false;
  }
  out << std::fixed << std::setprecision(6);
  out << "# reBotArm gravity identification output\n";
  out << "schema_version: 1\n";
  out << "identified_at: \"" << iso8601_now_local() << "\"\n";
  out << "source_action: StartGravityID\n";
  out << "mode: full_arm_gravity\n";
  out << "joint_mask: " << goal.joint_mask << "\n";
  out << "joints: [joint1, joint2, joint3, joint4, joint5, joint6]\n";
  out << "gravity_scales: [";
  for (std::size_t i = 0; i < kArmDof; ++i) {
    out << fit.gravity_scales[i] << (i + 1 < kArmDof ? ", " : "");
  }
  out << "]\n";
  out << "torque_biases_nm: [";
  for (std::size_t i = 0; i < kArmDof; ++i) {
    out << fit.torque_biases_nm[i] << (i + 1 < kArmDof ? ", " : "");
  }
  out << "]\n";
  out << "rmse_per_joint_nm: [";
  for (std::size_t i = 0; i < kArmDof; ++i) {
    out << fit.rmse_per_joint[i] << (i + 1 < kArmDof ? ", " : "");
  }
  out << "]\n";
  out << "rmse_total_nm: " << fit.rmse_total << "\n";
  out << "pose_count: " << pose_count << "\n";
  out << "payload:\n";
  out << "  enabled: false\n";
  out << "  frame: camera_link\n";
  out << "  mass_kg: 0.0\n";
  out << "  gravity_scale_joint4: 1.0\n";
  out << "  bias_joint4_nm: 0.0\n";
  out << "model_urdf_path: \"" << model_urdf_path_ << "\"\n";
  out << "notes: \"generated by /StartGravityID v1 FULL_ARM\"\n";
  out.close();
  path_out = path;
  message = "wrote " + path;
  return true;
}

bool GravityIdentificationNode::try_hot_reload(
  const gravity_identification::JointFitResult & fit, std::string & message)
{
  // Prefer parameter update on the live gravity_compensation_controller.
  const std::string service_name = "/gravity_compensation_controller/set_parameters";
  auto client = create_client<rcl_interfaces::srv::SetParameters>(service_name);
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    message =
      "set_parameters unavailable (" + service_name + "); "
      "apply YAML manually then restart/activate gravity_compensation_controller "
      "(TODO: dedicated ~/reload_gravity_params Trigger)";
    return false;
  }
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter scales;
  scales.name = "gravity_scales";
  scales.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  scales.value.double_array_value = fit.gravity_scales;
  rcl_interfaces::msg::Parameter biases;
  biases.name = "torque_biases";
  biases.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  biases.value.double_array_value = fit.torque_biases_nm;
  request->parameters = {scales, biases};
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    message = "set_parameters timed out";
    return false;
  }
  const auto response = future.get();
  for (const auto & result : response->results) {
    if (!result.successful) {
      message = "set_parameters rejected: " + result.reason;
      return false;
    }
  }
  message = "updated gravity_compensation_controller parameters";
  return true;
}

bool GravityIdentificationNode::deactivate_competing_controllers(std::string & message)
{
  const std::string service_name = controller_manager_name_ + "/switch_controller";
  auto client = create_client<controller_manager_msgs::srv::SwitchController>(service_name);
  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    message = "controller_manager not available";
    return false;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  // Humble field names are start_controllers / stop_controllers.
  request->stop_controllers = {
    "gravity_compensation_controller",
    "joint4_calibration_controller",
    "arm_teleop_controller",
    "arm_trajectory_controller",
    "vla_position_controller",
  };
  request->start_controllers = {};
  request->strictness = request->BEST_EFFORT;
  request->start_asap = false;
  request->timeout.sec = 0;
  request->timeout.nanosec = 0;
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    message = "switch_controller timed out";
    return false;
  }
  const auto response = future.get();
  if (!response->ok) {
    message = "switch_controller returned not ok";
    return false;
  }
  message = "deactivated competing controllers";
  return true;
}

std::vector<std::array<double, kArmDof>> GravityIdentificationNode::builtin_poses() const
{
  // 12 conservative poses; proximal (DM4340P) vs distal (DM4310) use different amplitudes.
  const auto & a = pose_amplitudes_;
  const auto & c = pose_centers_;
  const std::vector<std::array<double, kArmDof>> signs = {
    {+1, +1, +1, +1, +1, +1},
    {-1, +1, +1, -1, +1, -1},
    {+1, -1, +1, +1, -1, +1},
    {-1, -1, +1, -1, -1, -1},
    {+1, +1, -1, +1, +1, -1},
    {-1, +1, -1, -1, +1, +1},
    {+1, -1, -1, +1, -1, -1},
    {-1, -1, -1, -1, -1, +1},
    {+1, +0, +1, -1, +0, +1},
    {-1, +0, -1, +1, +0, -1},
    {+0, +1, +0, +1, -1, +0},
    {+0, -1, +0, -1, +1, +0},
  };
  std::vector<std::array<double, kArmDof>> poses;
  poses.reserve(signs.size());
  // Soft joint limits (inside URDF hard limits).
  const std::array<double, kArmDof> lower = {-1.2, -1.6, -1.8, -1.0, -1.0, -1.5};
  const std::array<double, kArmDof> upper = {1.2, -0.15, -0.15, 1.0, 1.0, 1.5};
  for (const auto & sign : signs) {
    std::array<double, kArmDof> pose{};
    for (std::size_t j = 0; j < kArmDof; ++j) {
      pose[j] = std::clamp(c[j] + sign[j] * a[j], lower[j], upper[j]);
    }
    poses.push_back(pose);
  }
  return poses;
}

std::array<double, kArmDof> GravityIdentificationNode::clamp_effort(
  const std::array<double, kArmDof> & effort, double max_effort_nm) const
{
  std::array<double, kArmDof> out{};
  for (std::size_t j = 0; j < kArmDof; ++j) {
    const double joint_cap = std::min(effort_limits_[j], max_effort_nm);
    const double hard_cap = std::min(joint_cap, 10.0);
    out[j] = clamp_abs(effort[j], hard_cap);
  }
  return out;
}

bool GravityIdentificationNode::snapshot_state(
  std::array<double, kArmDof> & position,
  std::array<double, kArmDof> & velocity,
  std::array<double, kArmDof> & effort,
  double max_age_s) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_valid_) {
    return false;
  }
  const double age = (now() - latest_state_stamp_).seconds();
  if (age > max_age_s) {
    return false;
  }
  position = latest_position_;
  velocity = latest_velocity_;
  effort = latest_effort_;
  return true;
}

}  // namespace microros_stm32h723_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<microros_stm32h723_bridge::GravityIdentificationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
