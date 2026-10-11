#ifndef MICROROS_STM32H723_BRIDGE__JOINT4_CALIBRATION_CONTROLLER_HPP_
#define MICROROS_STM32H723_BRIDGE__JOINT4_CALIBRATION_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "microros_stm32h723_bridge/gravity_compensation.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace microros_stm32h723_bridge
{

class Joint4CalibrationController : public controller_interface::ControllerInterface
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
  struct Sample
  {
    double position{0.0};
    double model_effort{0.0};
    double required_effort{0.0};
    double payload_sensitivity{0.0};
  };

  struct LinearFit
  {
    bool valid{false};
    double slope{0.0};
    double intercept{0.0};
    double rmse{0.0};
    double span{0.0};
  };

  void begin_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void release_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void capture_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void fit_samples(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void reset_samples(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void reset_stability_accumulator();
  LinearFit linear_fit(
    const std::vector<double> & x, const std::vector<double> & y,
    double minimum_span) const;

  GravityCompensation gravity_;
  std::vector<std::string> joints_;
  std::vector<double> gravity_scales_;
  std::vector<double> effort_limits_;
  std::string model_urdf_path_;
  std::string calibration_joint_{"joint4"};
  std::string payload_frame_{"camera_link"};
  std::size_t calibration_index_{0};
  double nominal_payload_mass_{0.075};
  double hold_position_gain_{8.0};
  double hold_velocity_gain_{0.5};
  double max_correction_effort_{1.5};
  double max_position_error_{0.12};
  double max_calibration_velocity_{0.20};
  double max_begin_velocity_{0.05};
  double capture_velocity_threshold_{0.01};
  double stable_duration_{1.5};
  double minimum_gravity_span_{0.25};
  double minimum_payload_sensitivity_span_{0.05};
  double status_publish_rate_{10.0};

  std::mutex state_mutex_;
  bool pose_active_{false};
  bool pose_faulted_{false};
  bool pose_captured_{false};
  bool state_valid_{false};
  double pose_reference_{0.0};
  double latest_position_{0.0};
  double latest_velocity_{0.0};
  double latest_model_effort_{0.0};
  double latest_required_effort_{0.0};
  double latest_payload_sensitivity_{0.0};
  double latest_correction_{0.0};
  double stable_elapsed_{0.0};
  double stable_position_sum_{0.0};
  double stable_model_effort_sum_{0.0};
  double stable_required_effort_sum_{0.0};
  double stable_payload_sensitivity_sum_{0.0};
  std::size_t stable_sample_count_{0};
  std::vector<Sample> samples_;
  std::atomic<bool> controller_active_{false};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr begin_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_publisher_;
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace microros_stm32h723_bridge

#endif  // MICROROS_STM32H723_BRIDGE__JOINT4_CALIBRATION_CONTROLLER_HPP_
