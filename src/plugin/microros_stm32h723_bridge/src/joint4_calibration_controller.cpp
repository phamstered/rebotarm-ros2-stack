#include "microros_stm32h723_bridge/joint4_calibration_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace microros_stm32h723_bridge
{

controller_interface::CallbackReturn Joint4CalibrationController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    auto_declare<std::string>("model_urdf_path", "");
    auto_declare<std::vector<double>>("gravity_scales", std::vector<double>(6, 1.0));
    auto_declare<std::vector<double>>(
      "effort_limits", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    auto_declare<std::string>("calibration_joint", "joint4");
    auto_declare<std::string>("payload_frame", "camera_link");
    auto_declare<double>("nominal_payload_mass", 0.075);
    auto_declare<double>("hold_position_gain", 8.0);
    auto_declare<double>("hold_velocity_gain", 0.5);
    auto_declare<double>("max_correction_effort", 1.5);
    auto_declare<double>("max_position_error", 0.12);
    auto_declare<double>("max_calibration_velocity", 0.20);
    auto_declare<double>("max_begin_velocity", 0.05);
    auto_declare<double>("capture_velocity_threshold", 0.01);
    auto_declare<double>("stable_duration", 1.5);
    auto_declare<double>("minimum_gravity_span", 0.25);
    auto_declare<double>("minimum_payload_sensitivity_span", 0.05);
    auto_declare<double>("status_publish_rate", 10.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Joint4CalibrationController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  model_urdf_path_ = get_node()->get_parameter("model_urdf_path").as_string();
  gravity_scales_ = get_node()->get_parameter("gravity_scales").as_double_array();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  calibration_joint_ = get_node()->get_parameter("calibration_joint").as_string();
  payload_frame_ = get_node()->get_parameter("payload_frame").as_string();
  nominal_payload_mass_ = get_node()->get_parameter("nominal_payload_mass").as_double();
  hold_position_gain_ = get_node()->get_parameter("hold_position_gain").as_double();
  hold_velocity_gain_ = get_node()->get_parameter("hold_velocity_gain").as_double();
  max_correction_effort_ = get_node()->get_parameter("max_correction_effort").as_double();
  max_position_error_ = get_node()->get_parameter("max_position_error").as_double();
  max_calibration_velocity_ =
    get_node()->get_parameter("max_calibration_velocity").as_double();
  max_begin_velocity_ = get_node()->get_parameter("max_begin_velocity").as_double();
  capture_velocity_threshold_ =
    get_node()->get_parameter("capture_velocity_threshold").as_double();
  stable_duration_ = get_node()->get_parameter("stable_duration").as_double();
  minimum_gravity_span_ = get_node()->get_parameter("minimum_gravity_span").as_double();
  minimum_payload_sensitivity_span_ =
    get_node()->get_parameter("minimum_payload_sensitivity_span").as_double();
  status_publish_rate_ = get_node()->get_parameter("status_publish_rate").as_double();

  if (joints_.empty() || gravity_scales_.size() != joints_.size() ||
    effort_limits_.size() != joints_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "joints, gravity_scales and effort_limits must have the same non-zero length");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto calibration_joint =
    std::find(joints_.begin(), joints_.end(), calibration_joint_);
  if (calibration_joint == joints_.end()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Calibration joint is not configured: %s",
      calibration_joint_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  calibration_index_ = static_cast<std::size_t>(
    std::distance(joints_.begin(), calibration_joint));

  const bool parameters_valid =
    nominal_payload_mass_ >= 0.0 && hold_position_gain_ > 0.0 &&
    hold_velocity_gain_ >= 0.0 && max_correction_effort_ > 0.0 &&
    max_position_error_ > 0.0 && max_calibration_velocity_ > 0.0 &&
    max_begin_velocity_ > 0.0 && capture_velocity_threshold_ > 0.0 &&
    capture_velocity_threshold_ <= max_begin_velocity_ && stable_duration_ > 0.0 &&
    minimum_gravity_span_ > 0.0 && minimum_payload_sensitivity_span_ > 0.0 &&
    status_publish_rate_ > 0.0 &&
    std::all_of(
      effort_limits_.begin(), effort_limits_.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;});
  if (!parameters_valid) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint4 calibration parameters are invalid");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (model_urdf_path_.empty()) {
    model_urdf_path_ =
      ament_index_cpp::get_package_share_directory("rebotarm_gazebo") + "/urdf/rebotarm.urdf";
  }
  try {
    gravity_.initialize(model_urdf_path_, joints_);
    const std::vector<double> zero_positions(joints_.size(), 0.0);
    gravity_.compute_point_mass_gravity_sensitivity(zero_positions, payload_frame_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Calibration model initialization failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  begin_pose_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/begin_pose", std::bind(&Joint4CalibrationController::begin_pose, this, _1, _2));
  release_pose_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/release_pose", std::bind(&Joint4CalibrationController::release_pose, this, _1, _2));
  capture_pose_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/capture_pose", std::bind(&Joint4CalibrationController::capture_pose, this, _1, _2));
  fit_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/fit", std::bind(&Joint4CalibrationController::fit_samples, this, _1, _2));
  reset_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&Joint4CalibrationController::reset_samples, this, _1, _2));
  status_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/status", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(), "Using gravity model %s; calibrating %s with payload frame %s",
    model_urdf_path_.c_str(), calibration_joint_.c_str(), payload_frame_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Joint4CalibrationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
Joint4CalibrationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_) {
    configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::CallbackReturn Joint4CalibrationController::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pose_active_ = false;
    pose_faulted_ = false;
    pose_captured_ = false;
    state_valid_ = false;
    latest_correction_ = 0.0;
    reset_stability_accumulator();
  }
  last_status_publish_time_ = get_node()->now();
  controller_active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Joint4CalibrationController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_.store(false);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pose_active_ = false;
    state_valid_ = false;
    latest_correction_ = 0.0;
    reset_stability_accumulator();
  }
  for (auto & command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      command_interface.set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Joint4CalibrationController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<double> positions(joints_.size(), 0.0);
  std::vector<double> velocities(joints_.size(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    positions[i] = state_interfaces_[i].get_value();
    velocities[i] = state_interfaces_[joints_.size() + i].get_value();
    if (!std::isfinite(positions[i]) || !std::isfinite(velocities[i])) {
      return controller_interface::return_type::ERROR;
    }
  }

  std::vector<double> gravity_effort;
  std::vector<double> payload_sensitivity;
  try {
    gravity_effort = gravity_.compute(positions);
    payload_sensitivity =
      gravity_.compute_point_mass_gravity_sensitivity(positions, payload_frame_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Calibration dynamics failed: %s", error.what());
    return controller_interface::return_type::ERROR;
  }

  const double dt = std::clamp(period.seconds(), 0.0, 0.1);
  std::vector<double> commands(joints_.size(), 0.0);
  bool pose_faulted_now = false;
  std_msgs::msg::Float64MultiArray status;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_position_ = positions[calibration_index_];
    latest_velocity_ = velocities[calibration_index_];
    latest_model_effort_ = gravity_effort[calibration_index_];
    latest_payload_sensitivity_ = payload_sensitivity[calibration_index_];
    state_valid_ = true;

    double correction = 0.0;
    if (pose_active_) {
      const double position_error = pose_reference_ - latest_position_;
      if (std::abs(position_error) > max_position_error_ ||
        std::abs(latest_velocity_) > max_calibration_velocity_)
      {
        pose_active_ = false;
        pose_faulted_ = true;
        pose_faulted_now = true;
        reset_stability_accumulator();
      } else {
        correction = std::clamp(
          hold_position_gain_ * position_error - hold_velocity_gain_ * latest_velocity_,
          -max_correction_effort_, max_correction_effort_);
      }
    }

    bool target_saturated = false;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      double requested = gravity_effort[i] * gravity_scales_[i];
      if (i == calibration_index_) {
        requested += correction;
        target_saturated = std::abs(requested) > effort_limits_[i];
      }
      commands[i] = std::clamp(requested, -effort_limits_[i], effort_limits_[i]);
    }
    latest_correction_ = correction;
    latest_required_effort_ = commands[calibration_index_];

    const bool correction_saturated =
      std::abs(correction) >= max_correction_effort_ - 1e-9;
    const bool stable =
      pose_active_ && !target_saturated && !correction_saturated &&
      std::abs(latest_velocity_) <= capture_velocity_threshold_;
    if (stable) {
      stable_elapsed_ += dt;
      stable_position_sum_ += latest_position_;
      stable_model_effort_sum_ += latest_model_effort_;
      stable_required_effort_sum_ += latest_required_effort_;
      stable_payload_sensitivity_sum_ += latest_payload_sensitivity_;
      ++stable_sample_count_;
    } else {
      reset_stability_accumulator();
    }

    status.data = {
      latest_position_, latest_velocity_, latest_model_effort_, latest_correction_,
      latest_required_effort_, latest_payload_sensitivity_, pose_reference_, stable_elapsed_,
      static_cast<double>(samples_.size()), pose_active_ ? 1.0 : 0.0,
      pose_faulted_ ? 1.0 : 0.0, pose_captured_ ? 1.0 : 0.0};
  }

  for (std::size_t i = 0; i < commands.size(); ++i) {
    command_interfaces_[i].set_value(commands[i]);
  }
  if (pose_faulted_now) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Calibration pose released: joint4 exceeded the configured velocity or displacement guard");
  }
  if (status_publisher_ &&
    (time - last_status_publish_time_).seconds() >= 1.0 / status_publish_rate_)
  {
    status_publisher_->publish(status);
    last_status_publish_time_ = time;
  }
  return controller_interface::return_type::OK;
}

void Joint4CalibrationController::begin_pose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!controller_active_.load()) {
    response->success = false;
    response->message = "controller is not active";
    return;
  }
  if (!state_valid_ || !std::isfinite(latest_position_) || !std::isfinite(latest_velocity_) ||
    std::abs(latest_velocity_) > max_begin_velocity_)
  {
    response->success = false;
    response->message = "support joint4 and wait until its velocity is below max_begin_velocity";
    return;
  }
  pose_reference_ = latest_position_;
  pose_active_ = true;
  pose_faulted_ = false;
  pose_captured_ = false;
  reset_stability_accumulator();

  std::ostringstream message;
  message << std::fixed << std::setprecision(5)
          << "pose started at joint4=" << pose_reference_
          << " rad; keep the arm clear while the soft hold settles";
  response->success = true;
  response->message = message.str();
}

void Joint4CalibrationController::release_pose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  pose_active_ = false;
  pose_faulted_ = false;
  pose_captured_ = false;
  latest_correction_ = 0.0;
  reset_stability_accumulator();
  response->success = true;
  response->message =
    "soft hold released and pose fault cleared; gravity feed-forward remains active";
}

void Joint4CalibrationController::capture_pose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!controller_active_.load() || !pose_active_ || pose_faulted_) {
    response->success = false;
    response->message = "no healthy calibration pose is active";
    return;
  }
  if (pose_captured_) {
    response->success = false;
    response->message = "this pose was already captured; move the arm and call begin_pose again";
    return;
  }
  if (stable_elapsed_ < stable_duration_ || stable_sample_count_ == 0) {
    std::ostringstream message;
    message << std::fixed << std::setprecision(2)
            << "pose is not stable: " << stable_elapsed_ << "/" << stable_duration_
            << " seconds";
    response->success = false;
    response->message = message.str();
    return;
  }

  const double count = static_cast<double>(stable_sample_count_);
  Sample sample;
  sample.position = stable_position_sum_ / count;
  sample.model_effort = stable_model_effort_sum_ / count;
  sample.required_effort = stable_required_effort_sum_ / count;
  sample.payload_sensitivity = stable_payload_sensitivity_sum_ / count;
  samples_.push_back(sample);
  pose_captured_ = true;

  std::ostringstream message;
  message << std::fixed << std::setprecision(5)
          << "captured sample " << samples_.size() << ": q4=" << sample.position
          << ", model=" << sample.model_effort
          << " Nm, required=" << sample.required_effort
          << " Nm, payload_sensitivity=" << sample.payload_sensitivity << " Nm/kg";
  response->success = true;
  response->message = message.str();
}

void Joint4CalibrationController::fit_samples(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::vector<Sample> samples;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    samples = samples_;
  }
  if (samples.size() < 4) {
    response->success = false;
    response->message = "at least four distinct stable poses are required";
    return;
  }

  std::vector<double> gravity_x;
  std::vector<double> gravity_y;
  std::vector<double> payload_x;
  std::vector<double> payload_y;
  gravity_x.reserve(samples.size());
  gravity_y.reserve(samples.size());
  payload_x.reserve(samples.size());
  payload_y.reserve(samples.size());
  for (const auto & sample : samples) {
    gravity_x.push_back(sample.model_effort);
    gravity_y.push_back(sample.required_effort);
    payload_x.push_back(sample.payload_sensitivity);
    const double effort_without_nominal_payload =
      sample.model_effort - nominal_payload_mass_ * sample.payload_sensitivity;
    payload_y.push_back(sample.required_effort - effort_without_nominal_payload);
  }

  const auto gravity_fit = linear_fit(gravity_x, gravity_y, minimum_gravity_span_);
  const auto payload_fit =
    linear_fit(payload_x, payload_y, minimum_payload_sensitivity_span_);
  if (!gravity_fit.valid && !payload_fit.valid) {
    response->success = false;
    response->message =
      "pose set is not observable; use poses with a wider joint4 gravity-moment range";
    return;
  }

  std::ostringstream message;
  message << std::fixed << std::setprecision(6) << "samples=" << samples.size();
  if (gravity_fit.valid) {
    message << "; gravity_scale=" << gravity_fit.slope
            << "; constant_bias=" << gravity_fit.intercept
            << " Nm; gravity_rmse=" << gravity_fit.rmse << " Nm";
  } else {
    message << "; gravity fit unavailable (insufficient model-torque span)";
  }
  if (payload_fit.valid) {
    message << "; equivalent_payload_mass=" << payload_fit.slope
            << " kg; payload_bias=" << payload_fit.intercept
            << " Nm; payload_rmse=" << payload_fit.rmse << " Nm";
  } else {
    message << "; payload fit unavailable (insufficient sensitivity span)";
  }
  response->success = true;
  response->message = message.str();
}

void Joint4CalibrationController::reset_samples(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  samples_.clear();
  pose_captured_ = false;
  reset_stability_accumulator();
  response->success = true;
  response->message = "all captured calibration samples were cleared";
}

void Joint4CalibrationController::reset_stability_accumulator()
{
  stable_elapsed_ = 0.0;
  stable_position_sum_ = 0.0;
  stable_model_effort_sum_ = 0.0;
  stable_required_effort_sum_ = 0.0;
  stable_payload_sensitivity_sum_ = 0.0;
  stable_sample_count_ = 0;
}

Joint4CalibrationController::LinearFit Joint4CalibrationController::linear_fit(
  const std::vector<double> & x, const std::vector<double> & y,
  double minimum_span) const
{
  LinearFit fit;
  if (x.size() != y.size() || x.size() < 2) {
    return fit;
  }
  const auto limits = std::minmax_element(x.begin(), x.end());
  fit.span = *limits.second - *limits.first;
  if (!std::isfinite(fit.span) || fit.span < minimum_span) {
    return fit;
  }

  const double count = static_cast<double>(x.size());
  const double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / count;
  const double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / count;
  double denominator = 0.0;
  double numerator = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double centered_x = x[i] - mean_x;
    denominator += centered_x * centered_x;
    numerator += centered_x * (y[i] - mean_y);
  }
  if (!std::isfinite(denominator) || denominator <= std::numeric_limits<double>::epsilon()) {
    return fit;
  }
  fit.slope = numerator / denominator;
  fit.intercept = mean_y - fit.slope * mean_x;
  double squared_error = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double residual = y[i] - (fit.slope * x[i] + fit.intercept);
    squared_error += residual * residual;
  }
  fit.rmse = std::sqrt(squared_error / count);
  fit.valid = std::isfinite(fit.slope) && std::isfinite(fit.intercept) &&
    std::isfinite(fit.rmse);
  return fit;
}

}  // namespace microros_stm32h723_bridge

PLUGINLIB_EXPORT_CLASS(
  microros_stm32h723_bridge::Joint4CalibrationController,
  controller_interface::ControllerInterface)
