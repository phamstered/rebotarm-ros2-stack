#ifndef MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_HPP_
#define MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_HPP_

#include <memory>
#include <string>
#include <vector>

namespace microros_stm32h723_bridge
{

class GravityCompensation
{
public:
  GravityCompensation();
  ~GravityCompensation();

  void initialize(
    const std::string & urdf_path,
    const std::vector<std::string> & hardware_joint_names);

  std::vector<double> compute(const std::vector<double> & hardware_positions);
  std::vector<double> compute_point_mass_gravity_sensitivity(
    const std::vector<double> & hardware_positions, const std::string & frame_name);
  bool initialized() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace microros_stm32h723_bridge

#endif  // MICROROS_STM32H723_BRIDGE__GRAVITY_COMPENSATION_HPP_
