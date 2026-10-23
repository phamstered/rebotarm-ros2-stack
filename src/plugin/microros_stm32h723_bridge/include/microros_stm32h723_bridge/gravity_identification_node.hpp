// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "microros_stm32h723_bridge/gravity_compensation.hpp"
#include "microros_stm32h723_bridge/gravity_identification_constants.hpp"
#include "microros_stm32h723_bridge/gravity_identification_fit.hpp"
#include "microros_stm32h723_bridge/action/gravity_identification.hpp"
#include "microros_stm32h723_bridge/msg/hardware_status.hpp"
#include "microros_stm32h723_bridge/msg/joint_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace microros_stm32h723_bridge
{

class GravityIdentificationNode : public rclcpp::Node
{
public:
  using GravityIdentification = action::GravityIdentification;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GravityIdentification>;

  explicit GravityIdentificationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct PoseSample
  {
    std::array<double, gravity_identification::kArmDof> position{};
    std::array<double, gravity_identification::kArmDof> measured_effort{};
    std::array<double, gravity_identification::kArmDof> model_effort{};
  };

  struct SessionGoal
  {
    uint32_t joint_mask{gravity_identification::DEFAULT_JOINT_MASK};
    uint8_t mode{gravity_identification::MODE_FULL_ARM_GRAVITY};
    double settle_time_s{1.5};
    double capture_window_s{1.0};
    bool write_yaml{true};
    bool hot_reload{false};
    std::string output_yaml_path;
    double max_effort_nm{10.0};
    double max_velocity_rad_s{0.15};
    double command_timeout_ms{100.0};
    bool lock_gripper{true};
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GravityIdentification::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void status_callback(const msg::HardwareStatus::SharedPtr message);
  void command_timer_callback();

  bool arm_session(const SessionGoal & goal, std::string & message);
  void disarm_session();
  void publish_hold_command();
  void publish_track_command(
    const std::array<double, gravity_identification::kArmDof> & position,
    const std::array<double, gravity_identification::kArmDof> & velocity,
    const std::array<double, gravity_identification::kArmDof> & effort,
    bool include_gripper_hold);
  bool wait_until(
    double timeout_s,
    const std::function<bool()> & predicate,
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t phase,
    const std::string & waiting_message);
  bool move_to_pose(
    const std::array<double, gravity_identification::kArmDof> & target,
    const SessionGoal & goal,
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint16_t pose_index,
    uint16_t pose_count,
    std::string & message);
  bool settle_and_capture(
    const SessionGoal & goal,
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint16_t pose_index,
    uint16_t pose_count,
    PoseSample & sample,
    std::string & message);
  void abort_to_hold(const std::string & reason);
  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t phase,
    uint16_t pose_index,
    uint16_t pose_count,
    const std::string & message,
    const PoseSample * sample = nullptr,
    float rmse_running = 0.0F);
  bool write_yaml_file(
    const SessionGoal & goal,
    const gravity_identification::JointFitResult & fit,
    uint16_t pose_count,
    std::string & path_out,
    std::string & message);
  bool try_hot_reload(
    const gravity_identification::JointFitResult & fit, std::string & message);
  bool deactivate_competing_controllers(std::string & message);
  std::vector<std::array<double, gravity_identification::kArmDof>> builtin_poses() const;
  std::array<double, gravity_identification::kArmDof> clamp_effort(
    const std::array<double, gravity_identification::kArmDof> & effort,
    double max_effort_nm) const;
  bool snapshot_state(
    std::array<double, gravity_identification::kArmDof> & position,
    std::array<double, gravity_identification::kArmDof> & velocity,
    std::array<double, gravity_identification::kArmDof> & effort,
    double max_age_s) const;

  GravityCompensation gravity_;
  rclcpp_action::Server<GravityIdentification>::SharedPtr action_server_;
  rclcpp::Publisher<msg::JointCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<msg::HardwareStatus>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  mutable std::mutex state_mutex_;
  std::array<double, gravity_identification::kArmDof> latest_position_{};
  std::array<double, gravity_identification::kArmDof> latest_velocity_{};
  std::array<double, gravity_identification::kArmDof> latest_effort_{};
  rclcpp::Time latest_state_stamp_{0, 0, RCL_ROS_TIME};
  bool state_valid_{false};

  mutable std::mutex status_mutex_;
  msg::HardwareStatus latest_status_;
  bool status_valid_{false};

  std::mutex command_mutex_;
  uint8_t command_mode_{msg::JointCommand::MODE_HOLD};
  std::array<double, gravity_identification::kArmDof> command_position_{};
  std::array<double, gravity_identification::kArmDof> command_velocity_{};
  std::array<double, gravity_identification::kArmDof> command_effort_{};
  bool command_streaming_{false};
  bool gripper_locked_{false};
  double gripper_hold_position_{0.0};
  uint32_t command_sequence_{0};

  std::atomic<bool> session_active_{false};
  std::atomic<bool> abort_requested_{false};
  std::string abort_reason_;
  uint32_t active_joint_mask_{gravity_identification::DEFAULT_JOINT_MASK};

  std::vector<std::string> joint_names_;
  std::string model_urdf_path_;
  std::string command_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string default_yaml_path_;
  std::string controller_manager_name_;
  bool require_exclusive_bus_{true};
  bool attempt_controller_switch_{true};
  double command_period_s_{0.02};
  double position_tolerance_rad_{0.03};
  double velocity_settle_threshold_{0.02};
  double minimum_model_span_nm_{0.15};
  double state_max_age_s_{0.25};
  std::array<double, gravity_identification::kArmDof> effort_limits_{
    10.0, 10.0, 10.0, 6.0, 6.0, 6.0};
  std::array<double, gravity_identification::kArmDof> track_kp_{
    40.0, 40.0, 40.0, 15.0, 15.0, 15.0};
  std::array<double, gravity_identification::kArmDof> track_kd_{
    3.0, 3.0, 3.0, 1.5, 1.5, 1.5};
  std::array<double, gravity_identification::kArmDof> hold_kp_{
    20.0, 20.0, 20.0, 10.0, 10.0, 10.0};
  std::array<double, gravity_identification::kArmDof> hold_kd_{
    2.0, 2.0, 2.0, 1.5, 1.5, 1.5};
  // Proximal DM4340P vs distal DM4310: separate excitation amplitudes.
  std::array<double, gravity_identification::kArmDof> pose_amplitudes_{
    0.45, 0.40, 0.45, 0.30, 0.25, 0.35};
  std::array<double, gravity_identification::kArmDof> pose_centers_{
    0.0, -0.70, -0.80, 0.0, 0.0, 0.0};
  std::vector<double> prior_scales_;
  std::vector<double> prior_biases_;
};

}  // namespace microros_stm32h723_bridge
