#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "microros_stm32h723_bridge/gravity_identification_fit.hpp"

using microros_stm32h723_bridge::gravity_identification::DEFAULT_JOINT_MASK;
using microros_stm32h723_bridge::gravity_identification::fit_per_joint;
using microros_stm32h723_bridge::gravity_identification::joint_active;
using microros_stm32h723_bridge::gravity_identification::kArmDof;
using microros_stm32h723_bridge::gravity_identification::linear_fit;
using microros_stm32h723_bridge::gravity_identification::resolve_joint_mask;

TEST(GravityIdentificationFit, ResolveMaskDefaultsToFullArm)
{
  EXPECT_EQ(resolve_joint_mask(0), DEFAULT_JOINT_MASK);
  EXPECT_EQ(resolve_joint_mask(0x3F), DEFAULT_JOINT_MASK);
  EXPECT_EQ(resolve_joint_mask(0x05), 0x05u);
  EXPECT_EQ(resolve_joint_mask(0xFF), DEFAULT_JOINT_MASK);
}

TEST(GravityIdentificationFit, JointActiveBits)
{
  EXPECT_TRUE(joint_active(0x3F, 0));
  EXPECT_TRUE(joint_active(0x3F, 5));
  EXPECT_FALSE(joint_active(0x3F, 6));
  EXPECT_TRUE(joint_active(0x04, 2));
  EXPECT_FALSE(joint_active(0x04, 1));
}

TEST(GravityIdentificationFit, LinearFitRecoversSlopeBias)
{
  std::vector<double> x = {-1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
  std::vector<double> y;
  y.reserve(x.size());
  constexpr double slope = 1.25;
  constexpr double bias = -0.15;
  for (double xi : x) {
    y.push_back(slope * xi + bias);
  }
  const auto fit = linear_fit(x, y, 0.1);
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.slope, slope, 1e-9);
  EXPECT_NEAR(fit.intercept, bias, 1e-9);
  EXPECT_NEAR(fit.rmse, 0.0, 1e-9);
}

TEST(GravityIdentificationFit, LinearFitRejectsLowSpan)
{
  std::vector<double> x = {1.0, 1.01, 1.02};
  std::vector<double> y = {2.0, 2.02, 2.04};
  const auto fit = linear_fit(x, y, 0.5);
  EXPECT_FALSE(fit.valid);
}

TEST(GravityIdentificationFit, FitPerJointUsesMaskAndPriors)
{
  std::vector<std::vector<double>> model(kArmDof);
  std::vector<std::vector<double>> meas(kArmDof);
  for (std::size_t pose = 0; pose < 8; ++pose) {
    for (std::size_t j = 0; j < kArmDof; ++j) {
      const double tau = -1.0 + 0.3 * static_cast<double>(pose) + 0.05 * static_cast<double>(j);
      model[j].push_back(tau);
      meas[j].push_back(1.1 * tau + 0.05 * static_cast<double>(j + 1));
    }
  }
  const std::vector<double> prior_scales(kArmDof, 1.0);
  const std::vector<double> prior_biases(kArmDof, 0.0);
  // Only joints 1 and 4 (bits 0 and 3).
  const auto result = fit_per_joint(0x09, model, meas, 0.1, prior_scales, prior_biases);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.gravity_scales[0], 1.1, 1e-6);
  EXPECT_NEAR(result.torque_biases_nm[0], 0.05, 1e-6);
  EXPECT_NEAR(result.gravity_scales[3], 1.1, 1e-6);
  EXPECT_NEAR(result.torque_biases_nm[3], 0.20, 1e-6);
  // Inactive joints keep priors.
  EXPECT_DOUBLE_EQ(result.gravity_scales[1], 1.0);
  EXPECT_DOUBLE_EQ(result.torque_biases_nm[1], 0.0);
}
