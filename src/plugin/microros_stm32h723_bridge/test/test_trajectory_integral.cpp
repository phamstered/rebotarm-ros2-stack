#include <gtest/gtest.h>
#include <limits>
#include "microros_stm32h723_bridge/trajectory_integral.hpp"

using microros_stm32h723_bridge::TrajectoryIntegral;

namespace
{
TrajectoryIntegral configured(double cap = 2.0, double rate = 0.1)
{
  TrajectoryIntegral result;
  TrajectoryIntegral::Values ki{}, caps{}, rates{}, limits{};
  ki[0] = 4.0; caps[0] = cap; rates[0] = rate; limits.fill(10.0);
  result.configure(ki, caps, rates, limits);
  return result;
}
}

TEST(TrajectoryIntegral, DisabledAndUnclaimedKeepOnlyBoundedOriginalEffort)
{
  TrajectoryIntegral trim;
  TrajectoryIntegral::Values zeros{}, limits{}; limits.fill(10.0);
  trim.configure(zeros, zeros, zeros, limits);
  EXPECT_DOUBLE_EQ(trim.update(0, true, 0.1, 1.0, .01), 1.0);
  EXPECT_DOUBLE_EQ(trim.update(0, true, 0.1, 30.0, .01), 10.0);
  trim = configured();
  EXPECT_GT(trim.update(0, true, .1, 1.0, .01), 1.0);
  EXPECT_DOUBLE_EQ(trim.update(0, false, .1, 1.0, .01), 1.0);
  EXPECT_DOUBLE_EQ(trim.value(0), 0.0);
}

TEST(TrajectoryIntegral, SignedCorrectionRespectsSlewAndMagnitudeCaps)
{
  auto trim = configured(.2, .05);
  for (unsigned i = 0; i < 1000; ++i) {
    const double before = trim.value(0);
    trim.update(0, true, .3, 0.0, .01);
    EXPECT_LE(trim.value(0) - before, .0005 + 1e-12);
  }
  EXPECT_NEAR(trim.value(0), .2, 1e-12);
  for (unsigned i = 0; i < 1000; ++i) {
    const double before = trim.value(0);
    trim.update(0, true, -.3, 0.0, .01);
    EXPECT_LE(before - trim.value(0), .0005 + 1e-12);
  }
  EXPECT_NEAR(trim.value(0), -.2, 1e-12);
}

TEST(TrajectoryIntegral, SaturationStopsAccumulationAndAllowsUnwinding)
{
  auto trim = configured();
  for (unsigned i = 0; i < 1000; ++i) {EXPECT_LE(trim.update(0, true, .1, 9.9, .01), 10.0);}
  EXPECT_LE(trim.value(0), .1 + 1e-12);
  const double before = trim.value(0);
  for (unsigned i = 0; i < 20; ++i) {trim.update(0, true, -.1, 9.9, .01);}
  EXPECT_LT(trim.value(0), before);
  for (unsigned i = 0; i < 1000; ++i) {EXPECT_GE(trim.update(0, true, -.1, -9.9, .01), -10.0);}
  EXPECT_GE(trim.value(0), -.1 - 1e-12);
}

TEST(TrajectoryIntegral, InvalidOrDelayedCycleNeverAppliesCatchUp)
{
  auto trim = configured();
  trim.update(0, true, .1, 0, .01);
  const double before = trim.value(0);
  for (double dt : {0.0, -.1, .051, 10.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    trim.update(0, true, .1, 0, dt);
    EXPECT_DOUBLE_EQ(trim.value(0), before);
  }
  EXPECT_THROW(trim.update(0, true, NAN, 0, .01), std::invalid_argument);
  EXPECT_THROW(trim.update(0, true, .1, INFINITY, .01), std::invalid_argument);
  trim.reset(); EXPECT_DOUBLE_EQ(trim.value(0), 0.0);
}

TEST(TrajectoryIntegral, InvalidConfigurationFailsBeforeActuation)
{
  TrajectoryIntegral trim;
  TrajectoryIntegral::Values ki{}, caps{}, rates{}, limits{}; limits.fill(10.0);
  ki[0] = 4.0;
  EXPECT_THROW(trim.configure(ki, caps, rates, limits), std::invalid_argument);
  caps[0] = 2.0; rates[0] = .1;
  for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    ki[0] = invalid;
    EXPECT_THROW(trim.configure(ki, caps, rates, limits), std::invalid_argument);
  }
  ki[0] = 4.0; caps[0] = 11.0;
  EXPECT_THROW(trim.configure(ki, caps, rates, limits), std::invalid_argument);
  caps[0] = 2.0; ki[6] = 1; caps[6] = 1; rates[6] = .1;
  EXPECT_THROW(trim.configure(ki, caps, rates, limits), std::invalid_argument);
}

TEST(TrajectoryIntegral, ConstantLoadModelConvergesWithoutWideningFiveMilliradianGate)
{
  // Quasi-static equilibrium only, not a gearbox/friction or real-arm model.
  for (double load : {.223, .848, -1.855}) {
    auto trim = configured(2.0, .1);
    double error = load / 40.0;
    for (unsigned i = 0; i < 6000; ++i) {
      const double effort = trim.update(0, true, error, 0.0, .01);
      error = (load - effort) / 40.0;
      EXPECT_LE(std::abs(trim.value(0)), 2.0);
    }
    EXPECT_LT(std::abs(error), .005);
  }
}
