#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mur_control/collision_response.hpp"

namespace
{

void expect_vector_near(const std::vector<double> & actual, const std::vector<double> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1.0e-12) << "index " << i;
  }
}

}  // namespace

TEST(CollisionResponse, ClearAboveActivationLeavesCommandUnchanged)
{
  const std::vector<double> qdot{0.1, -0.2, 0.3};
  const std::vector<double> gradient{1.0, 0.0, 0.0};

  const auto response = mur_control::limit_collision_qdot(
    qdot, gradient, 0.12, 0.035, 0.08, mur_control::CollisionResponseMode::Scale);

  EXPECT_EQ(response.status, "clear");
  EXPECT_DOUBLE_EQ(response.scale, 1.0);
  expect_vector_near(response.qdot, qdot);
}

TEST(CollisionResponse, MovingAwayInsideActivationLeavesCommandUnchanged)
{
  const std::vector<double> qdot{0.1, -0.2, 0.3};
  const std::vector<double> gradient{1.0, 0.0, 0.0};

  const auto response = mur_control::limit_collision_qdot(
    qdot, gradient, 0.05, 0.035, 0.08, mur_control::CollisionResponseMode::Scale);

  EXPECT_EQ(response.status, "clear");
  EXPECT_DOUBLE_EQ(response.scale, 1.0);
  EXPECT_GT(response.closing_speed, 0.0);
  expect_vector_near(response.qdot, qdot);
}

TEST(CollisionResponse, ScaleModePreservesDirectionWhileApproaching)
{
  const std::vector<double> qdot{-0.2, 0.1, 0.04};
  const std::vector<double> gradient{1.0, 0.0, 0.0};

  const auto response = mur_control::limit_collision_qdot(
    qdot, gradient, 0.0575, 0.035, 0.08, mur_control::CollisionResponseMode::Scale);

  EXPECT_EQ(response.status, "limited");
  EXPECT_NEAR(response.scale, 0.5, 1.0e-12);
  EXPECT_LT(response.closing_speed, 0.0);
  expect_vector_near(response.qdot, {-0.1, 0.05, 0.02});
}

TEST(CollisionResponse, StopClearanceBlocksCommand)
{
  const std::vector<double> qdot{-0.2, 0.1, 0.04};
  const std::vector<double> gradient{1.0, 0.0, 0.0};

  const auto response = mur_control::limit_collision_qdot(
    qdot, gradient, 0.03, 0.035, 0.08, mur_control::CollisionResponseMode::Scale);

  EXPECT_EQ(response.status, "blocked");
  EXPECT_DOUBLE_EQ(response.scale, 0.0);
  expect_vector_near(response.qdot, {0.0, 0.0, 0.0});
}

TEST(CollisionResponse, ProjectModeRemainsAvailableAsLegacyResponse)
{
  const std::vector<double> qdot{-0.2, 0.1, 0.04};
  const std::vector<double> gradient{1.0, 0.0, 0.0};

  const auto response = mur_control::limit_collision_qdot(
    qdot, gradient, 0.0575, 0.035, 0.08, mur_control::CollisionResponseMode::Project);

  EXPECT_EQ(response.status, "limited");
  expect_vector_near(response.qdot, {0.0, 0.1, 0.04});
}

TEST(CollisionResponse, ParsesResponseModeNames)
{
  EXPECT_EQ(
    mur_control::parse_collision_response_mode("scale"),
    mur_control::CollisionResponseMode::Scale);
  EXPECT_EQ(
    mur_control::parse_collision_response_mode("direction_preserving"),
    mur_control::CollisionResponseMode::Scale);
  EXPECT_EQ(
    mur_control::parse_collision_response_mode("project"),
    mur_control::CollisionResponseMode::Project);
  EXPECT_THROW(
    mur_control::parse_collision_response_mode("unknown_mode"),
    std::invalid_argument);
}
