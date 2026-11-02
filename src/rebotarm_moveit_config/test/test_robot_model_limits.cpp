// Read-only RobotModelLoader probe. Requires real launch parameters as a file;
// does not create controllers, Action clients, or motion goals.
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_group", rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .start_parameter_services(false).start_parameter_event_publisher(false));
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();
  if (!model) return 1;
  bool valid = true;
  for (const auto &name : {"joint1","joint2","joint3","joint4","joint5","joint6","gripper_finger_left_joint"}) {
    const auto joint = model->getJointModel(name);
    if (!joint || joint->getVariableBounds().size() != 1) { valid = false; continue; }
    const auto &b = joint->getVariableBounds().front();
    const bool grip = std::string(name) == "gripper_finger_left_joint";
    const double velocity = grip ? .02 : 2., acceleration = grip ? .04 : 2.;
    std::cout << name << " velocity=" << b.max_velocity_ << " acceleration=" << b.max_acceleration_
              << " bounded=" << b.velocity_bounded_ << "/" << b.acceleration_bounded_ << std::endl;
    valid = valid && b.velocity_bounded_ && b.acceleration_bounded_ &&
        std::abs(b.max_velocity_-velocity)<1e-9 && std::abs(b.min_velocity_+velocity)<1e-9 &&
        std::abs(b.max_acceleration_-acceleration)<1e-9 && std::abs(b.min_acceleration_+acceleration)<1e-9;
  }
  rclcpp::shutdown();
  return valid ? 0 : 1;
}
