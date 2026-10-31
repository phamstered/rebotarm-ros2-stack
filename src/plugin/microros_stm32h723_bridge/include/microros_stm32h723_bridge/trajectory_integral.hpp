#ifndef MICROROS_STM32H723_BRIDGE__TRAJECTORY_INTEGRAL_HPP_
#define MICROROS_STM32H723_BRIDGE__TRAJECTORY_INTEGRAL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace microros_stm32h723_bridge
{

// Optional effort trim around the existing motor PD loop. It is not a gravity
// identifier and must never change the actuator's existing effort limits.
class TrajectoryIntegral
{
public:
  using Values = std::array<double, 7>;

  void configure(const Values & ki, const Values & caps, const Values & rates,
    const Values & effort_limits)
  {
    for (std::size_t i = 0; i < ki.size(); ++i) {
      for (double value : {ki[i], caps[i], rates[i], effort_limits[i]}) {
        if (!std::isfinite(value) || value < 0.0) {
          throw std::invalid_argument("integral settings and effort limits must be finite and nonnegative");
        }
      }
      if ((ki[i] == 0.0) != (caps[i] == 0.0) || (ki[i] == 0.0) != (rates[i] == 0.0) ||
        caps[i] > effort_limits[i] || !std::isfinite(static_cast<float>(effort_limits[i])) ||
        (i == 6 && ki[i] != 0.0))
      {
        throw std::invalid_argument("integral requires Ki/cap/rate together, cap within effort limit, gripper disabled");
      }
    }
    ki_ = ki; caps_ = caps; rates_ = rates; limits_ = effort_limits;
    reset();
  }

  void reset() {values_.fill(0.0);}
  void reset(std::size_t index) {values_.at(index) = 0.0;}
  double value(std::size_t index) const {return values_.at(index);}

  double update(std::size_t index, bool active, double error, double base_effort, double dt)
  {
    if (!std::isfinite(error) || !std::isfinite(base_effort)) {
      throw std::invalid_argument("nonfinite integral input");
    }
    const double limit = limits_.at(index);
    const double base = std::clamp(base_effort, -limit, limit);
    if (!active || ki_[index] == 0.0) {
      reset(index);
      return base;
    }
    // A delayed cycle is not permission to apply a large catch-up correction.
    if (std::isfinite(dt) && dt > 0.0 && dt <= 0.05) {
      const double delta = std::clamp(ki_[index] * error * dt,
          -rates_[index] * dt, rates_[index] * dt);
      const double candidate = std::clamp(values_[index] + delta, -caps_[index], caps_[index]);
      // Conditional integration: permit unwinding, but do not accumulate
      // further into either the trim cap or the actuator effort saturation.
      if (!((delta > 0.0 && base + candidate > limit) ||
        (delta < 0.0 && base + candidate < -limit)))
      {
        values_[index] = candidate;
      }
    }
    return std::clamp(base + values_[index], -limit, limit);
  }

private:
  Values ki_{}, caps_{}, rates_{}, limits_{}, values_{};
};

}  // namespace microros_stm32h723_bridge
#endif
