#include <gtest/gtest.h>
#include <limits>
#include "mur_control/wrench_bias.hpp"

using mur_control::WrenchBias;

TEST(WrenchBias, RequiresCompleteStationaryWindow)
{
  WrenchBias bias;
  const WrenchBias::Sample load{10, 20, 30, 1, 2, 3};
  EXPECT_FALSE(bias.update(load, 0.0, 1.0, true));
  EXPECT_FALSE(bias.update(load, 0.5, 1.0, true));
  EXPECT_TRUE(bias.update(load, 1.0, 1.0, true));
  EXPECT_EQ(bias.value(), load);
}

TEST(WrenchBias, MovementAndInvalidSamplesRestartWindow)
{
  WrenchBias bias;
  WrenchBias::Sample load{10, 20, 30, 1, 2, 3};
  EXPECT_FALSE(bias.update(load, 0.0, 1.0, true));
  EXPECT_FALSE(bias.update(load, 0.9, 1.0, false));
  EXPECT_FALSE(bias.update(load, 1.0, 1.0, true));
  load[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(bias.update(load, 1.9, 1.0, true));
  load[0] = 42;
  EXPECT_FALSE(bias.update(load, 2.0, 1.0, true));
  EXPECT_FALSE(bias.update(load, 2.9, 1.0, true));
  EXPECT_TRUE(bias.update(load, 3.0, 1.0, true));
  EXPECT_EQ(bias.value(), load);
}

TEST(WrenchBias, ReactivationDoesNotReusePreviousBias)
{
  WrenchBias bias;
  EXPECT_FALSE(bias.update({1, 2, 3, 4, 5, 6}, 0, 1, true));
  EXPECT_TRUE(bias.update({1, 2, 3, 4, 5, 6}, 1, 1, true));
  bias.reset();
  const WrenchBias::Sample new_load{10, 20, 30, 40, 50, 60};
  EXPECT_FALSE(bias.update(new_load, 5, 1, true));
  EXPECT_TRUE(bias.update(new_load, 6, 1, true));
  EXPECT_EQ(bias.value(), new_load);
}
