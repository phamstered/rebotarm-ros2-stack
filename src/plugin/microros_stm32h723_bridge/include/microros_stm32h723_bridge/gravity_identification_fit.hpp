// SPDX-License-Identifier: MIT
// Pure fitting / mask helpers for /StartGravityID (no ROS types).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "microros_stm32h723_bridge/gravity_identification_constants.hpp"

namespace microros_stm32h723_bridge
{
namespace gravity_identification
{

constexpr std::size_t kArmDof = 6;

struct LinearFit
{
  bool valid{false};
  double slope{1.0};
  double intercept{0.0};
  double rmse{0.0};
  double span{0.0};
};

inline uint32_t resolve_joint_mask(uint32_t joint_mask)
{
  return joint_mask == 0U ? DEFAULT_JOINT_MASK : (joint_mask & DEFAULT_JOINT_MASK);
}

inline bool joint_active(uint32_t mask, std::size_t index)
{
  return index < kArmDof && ((mask >> index) & 1U) != 0U;
}

inline LinearFit linear_fit(
  const std::vector<double> & x, const std::vector<double> & y, double minimum_span)
{
  LinearFit fit;
  if (x.size() != y.size() || x.size() < 2) {
    return fit;
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(y[i])) {
      return fit;
    }
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

struct JointFitResult
{
  std::vector<double> gravity_scales;
  std::vector<double> torque_biases_nm;
  std::vector<double> rmse_per_joint;
  double rmse_total{0.0};
  std::string message;
  bool success{false};
};

// samples_model[j][pose], samples_meas[j][pose] for active joints only.
inline JointFitResult fit_per_joint(
  uint32_t joint_mask,
  const std::vector<std::vector<double>> & samples_model,
  const std::vector<std::vector<double>> & samples_meas,
  double minimum_span,
  const std::vector<double> & prior_scales,
  const std::vector<double> & prior_biases)
{
  JointFitResult result;
  result.gravity_scales = prior_scales.size() == kArmDof ?
    prior_scales : std::vector<double>(kArmDof, 1.0);
  result.torque_biases_nm = prior_biases.size() == kArmDof ?
    prior_biases : std::vector<double>(kArmDof, 0.0);
  result.rmse_per_joint.assign(kArmDof, 0.0);

  if (samples_model.size() != kArmDof || samples_meas.size() != kArmDof) {
    result.message = "internal sample buffers must be length 6";
    return result;
  }

  const uint32_t mask = resolve_joint_mask(joint_mask);
  double total_sq = 0.0;
  std::size_t total_count = 0;
  std::size_t fitted = 0;
  std::size_t failed = 0;

  for (std::size_t j = 0; j < kArmDof; ++j) {
    if (!joint_active(mask, j)) {
      continue;
    }
    const auto fit = linear_fit(samples_model[j], samples_meas[j], minimum_span);
    if (!fit.valid) {
      ++failed;
      result.rmse_per_joint[j] =
        std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    result.gravity_scales[j] = fit.slope;
    result.torque_biases_nm[j] = fit.intercept;
    result.rmse_per_joint[j] = fit.rmse;
    ++fitted;
    for (std::size_t i = 0; i < samples_model[j].size(); ++i) {
      const double residual =
        samples_meas[j][i] - (fit.slope * samples_model[j][i] + fit.intercept);
      total_sq += residual * residual;
      ++total_count;
    }
  }

  if (fitted == 0) {
    result.message =
      "no joint produced a valid fit; widen pose set or lower minimum_span";
    return result;
  }
  result.rmse_total = total_count > 0 ? std::sqrt(total_sq / static_cast<double>(total_count)) : 0.0;
  result.success = failed == 0;
  result.message = result.success ?
    ("fitted " + std::to_string(fitted) + " joints") :
    ("fitted " + std::to_string(fitted) + " joints; " + std::to_string(failed) +
    " failed observability check");
  return result;
}

}  // namespace gravity_identification
}  // namespace microros_stm32h723_bridge
