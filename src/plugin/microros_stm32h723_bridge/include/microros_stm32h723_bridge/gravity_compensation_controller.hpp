#ifndef MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "microros_stm32h723_bridge/gravity_compensation.hpp"

namespace microros_stm32h723_bridge
{

class GravityCompensationController : public controller_interface::ControllerInterface
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
  GravityCompensation gravity_;
  std::vector<std::string> joints_;
  std::vector<double> gravity_scales_;
  std::vector<double> effort_limits_;
  std::vector<double> startup_positions_;
  std::string model_urdf_path_;
  std::chrono::steady_clock::time_point activated_at_{};
  double startup_hold_duration_sec_{5.0};
};

}  // namespace microros_stm32h723_bridge

#endif  // MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_CONTROLLER_HPP_
