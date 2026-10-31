#ifndef MICROROS_STM32H723_BRIDGE__MICRO_ROS_SYSTEM_HPP_
#define MICROROS_STM32H723_BRIDGE__MICRO_ROS_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "microros_stm32h723_bridge/msg/hardware_status.hpp"
#include "microros_stm32h723_bridge/msg/joint_command.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace microros_stm32h723_bridge
{

class MicroRosSystem : public hardware_interface::SystemInterface
{
public:
  static constexpr std::size_t kJointCount = 7;

  ~MicroRosSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  friend struct MicroRosSystemTestAccess;

  struct Feedback
  {
    std::array<double, kJointCount> position{};
    std::array<double, kJointCount> velocity{};
    std::array<double, kJointCount> effort{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
  };

  struct Gains
  {
    std::array<double, kJointCount> kp{};
    std::array<double, kJointCount> kd{};
  };

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void status_callback(const msg::HardwareStatus::SharedPtr message);
  void request_safe_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void request_disable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_executor();
  bool load_gains(const std::string & path);
  bool all_finite(const std::array<double, kJointCount> & values) const;
  bool configured_positions_differ(
    const std::array<double, kJointCount> & lhs,
    const std::array<double, kJointCount> & rhs) const;
  bool configured_joints_at_zero() const;
  const Gains & selected_gains() const;

  std::array<double, kJointCount> state_position_{};
  std::array<double, kJointCount> state_velocity_{};
  std::array<double, kJointCount> state_effort_{};
  std::array<double, kJointCount> command_position_{};
  std::array<double, kJointCount> command_velocity_{};
  std::array<double, kJointCount> command_effort_{};
  std::array<bool, kJointCount> velocity_available_{};
  std::atomic<uint32_t> velocity_claimed_mask_{0U};
  std::atomic<uint32_t> position_claimed_mask_{0U};
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> effort_limits_{};
  std::array<double, kJointCount> position_scales_{};
  std::array<double, kJointCount> position_offsets_{};
  std::array<double, kJointCount> return_start_{};

  Gains hold_gains_;
  Gains track_gains_;
  Gains return_gains_;
  Gains idle_gains_;

  std::mutex feedback_mutex_;
  std::condition_variable feedback_cv_;
  Feedback feedback_;
  std::mutex status_mutex_;
  msg::HardwareStatus status_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<msg::JointCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<msg::HardwareStatus>::SharedPtr status_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safe_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::vector<std::string> joint_names_;
  std::vector<std::size_t> configured_protocol_indices_;
  std::atomic<uint8_t> command_mode_{msg::JointCommand::MODE_HOLD};
  std::atomic<bool> active_{false};
  std::atomic<bool> required_joints_active_{false};
  std::atomic<bool> safe_stop_requested_{false};
  std::atomic<bool> disable_requested_{false};
  std::chrono::steady_clock::time_point return_started_at_{};
  std::chrono::milliseconds state_timeout_{100};
  std::chrono::milliseconds activation_timeout_{3000};
  std::chrono::milliseconds return_duration_{3000};
  uint32_t sequence_{0};
  uint32_t disable_sequence_{0};
  uint32_t required_joint_mask_{0x7FU};
  uint32_t configured_joint_mask_{0U};
  double zero_tolerance_{0.03};
  std::string command_topic_{"/stm32/joint_commands"};
  std::string state_topic_{"/stm32/joint_states"};
  std::string status_topic_{"/stm32/status"};
};

}  // namespace microros_stm32h723_bridge

#endif  // MICROROS_STM32H723_BRIDGE__MICRO_ROS_SYSTEM_HPP_
