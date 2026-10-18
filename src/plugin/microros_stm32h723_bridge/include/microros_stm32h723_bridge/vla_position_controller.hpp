#ifndef MICROROS_STM32H723_BRIDGE__VLA_POSITION_CONTROLLER_HPP_
#define MICROROS_STM32H723_BRIDGE__VLA_POSITION_CONTROLLER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "microros_stm32h723_bridge/msg/vla_joint_chunk.hpp"
#include "rclcpp/subscription.hpp"

namespace microros_stm32h723_bridge
{

/// Position-only controller driven by external VLA joint chunks.
///
/// Claims joint*/position command interfaces (same as arm_teleop_controller /
/// arm_trajectory_controller). Run in parallel with
/// gravity_compensation_controller (effort). Do not activate together with
/// teleop or trajectory controllers.
class VlaPositionController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  void chunk_callback(const microros_stm32h723_bridge::msg::VlaJointChunk::SharedPtr message);

  std::vector<std::string> joints_;
  std::string chunk_topic_{"/vla/joint_chunk"};
  double command_timeout_sec_{0.2};
  double max_delta_per_sec_{1.0};

  std::mutex command_mutex_;
  std::vector<double> pending_target_;
  std::vector<double> last_command_;
  rclcpp::Time last_chunk_time_{0, 0, RCL_ROS_TIME};
  double active_timeout_sec_{0.2};
  bool has_valid_command_{false};
  bool hold_warned_{false};

  rclcpp::Subscription<microros_stm32h723_bridge::msg::VlaJointChunk>::SharedPtr chunk_subscription_;
};

}  // namespace microros_stm32h723_bridge

#endif  // MICROROS_STM32H723_BRIDGE__VLA_POSITION_CONTROLLER_HPP_
