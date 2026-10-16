#include <gtest/gtest.h>
#include "match_mur_navigation/mapping_core.hpp"

using mur_mapping::Grid;
using mur_mapping::PoseBuffer;

static int cell(const Grid &grid, double x, double y) {
  const auto i = static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution()));
  const auto j = static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()));
  return grid.data().at(static_cast<size_t>(j) * grid.width() + i);
}

TEST(Grid, WallFreeUnknownAndNegativeExpansion) {
  Grid grid(0.1, 1000000);
  for (int i = 0; i < 6; ++i) ASSERT_TRUE(grid.integrate({{0.05, 0.05, 1.05, 0.05, true}}));
  EXPECT_GE(cell(grid, 1.05, 0.05), 65);
  EXPECT_LE(cell(grid, 0.55, 0.05), 25);
  EXPECT_EQ(cell(grid, 0.55, 0.55), -1);
  ASSERT_TRUE(grid.integrate({{0.05, 0.05, -10.05, -10.05, true}}));
  EXPECT_LT(grid.origin_x(), -10);
  EXPECT_LT(grid.origin_y(), -10);
  EXPECT_GE(cell(grid, 1.05, 0.05), 65);
  EXPECT_EQ(cell(grid, -9.05, 1.05), -1);
}

TEST(Grid, NoReturnAndCorrelatedBeams) {
  Grid grid(0.1, 100000);
  std::vector<mur_mapping::Ray> rays(100, {0.05, 0.05, 2.05, 0.05, false});
  rays.push_back({0.05, 0.05, 1.05, 0.05, true});
  ASSERT_TRUE(grid.integrate(rays));
  EXPECT_GE(cell(grid, 1.05, 0.05), 65); // Endpoints win over beams crossing the same cell.
  EXPECT_GT(cell(grid, 0.55, 0.05), 25); // One scan is one observation, not 100 independent ones.
  for (int i = 0; i < 20; ++i) grid.integrate({{0.05, 0.05, 2.05, 0.05, false}});
  EXPECT_LE(cell(grid, 2.05, 0.05), 25);
}

TEST(Grid, LimitAndMalformedRaysDoNotChangeMap) {
  Grid grid(0.1, 4096);
  ASSERT_TRUE(grid.integrate({{0.05, 0.05, 1.05, 0.05, true}}));
  const auto before = grid.data();
  EXPECT_FALSE(grid.integrate({{0.05, 0.05, 1000, 1000, true}}));
  EXPECT_FALSE(grid.integrate({{NAN, 0, 1, 1, true}}));
  EXPECT_EQ(grid.data(), before);
}

TEST(Poses, TranslationFullRotationAndYawWrap) {
  PoseBuffer poses(0.2);
  tf2::Quaternion a, b;
  a.setRPY(0.2, 0.1, 179 * M_PI / 180);
  b.setRPY(0.2, 0.1, -179 * M_PI / 180);
  poses.add(1.0, tf2::Transform(a, tf2::Vector3(1, 2, 3)));
  poses.add(1.1, tf2::Transform(b, tf2::Vector3(3, 4, 5)));
  tf2::Transform result;
  ASSERT_TRUE(poses.at(1.05, result));
  EXPECT_NEAR(result.getOrigin().x(), 2, 1e-7);
  EXPECT_NEAR(result.getOrigin().z(), 4, 1e-7);
  EXPECT_NEAR(result.getRotation().angleShortestPath(a), M_PI / 180, 1e-7);
  EXPECT_FALSE(poses.at(0.99, result));
  EXPECT_FALSE(poses.at(1.11, result));
  poses.add(2.0, tf2::Transform::getIdentity());
  EXPECT_FALSE(poses.at(1.5, result));
}
