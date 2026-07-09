#pragma once

#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace mur_control
{

enum class CollisionResponseMode
{
  Scale,
  Project,
};

struct CollisionResponse
{
  std::vector<double> qdot;
  std::string status{"clear"};
  double scale{1.0};
  double closing_speed{0.0};
};

inline std::string collision_response_mode_name(CollisionResponseMode mode)
{
  switch (mode) {
    case CollisionResponseMode::Scale:
      return "scale";
    case CollisionResponseMode::Project:
      return "project";
  }
  return "scale";
}

inline CollisionResponseMode parse_collision_response_mode(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
  });
  if (value == "scale" || value == "limit" || value == "direction_preserving") {
    return CollisionResponseMode::Scale;
  }
  if (value == "project" || value == "projection" || value == "gradient_projection") {
    return CollisionResponseMode::Project;
  }
  throw std::invalid_argument(
    "Unsupported collision_response_mode '" + value + "'; expected 'scale' or 'project'");
}

inline double collision_clearance_scale(
  double clearance,
  double stop_clearance,
  double activation_clearance)
{
  if (!std::isfinite(clearance)) {
    return 0.0;
  }
  if (clearance <= stop_clearance || clearance < 0.0) {
    return 0.0;
  }
  if (clearance >= activation_clearance) {
    return 1.0;
  }
  const double span = activation_clearance - stop_clearance;
  if (span <= 1.0e-12) {
    return 0.0;
  }
  return std::clamp((clearance - stop_clearance) / span, 0.0, 1.0);
}

inline std::vector<double> zero_qdot_like(const std::vector<double> & qdot)
{
  return std::vector<double>(qdot.size(), 0.0);
}

inline CollisionResponse limit_collision_qdot(
  const std::vector<double> & qdot,
  const std::vector<double> & clearance_gradient,
  double clearance,
  double stop_clearance,
  double activation_clearance,
  CollisionResponseMode mode)
{
  CollisionResponse response;
  response.qdot = qdot;

  if (clearance >= activation_clearance) {
    response.status = "clear";
    response.scale = 1.0;
    return response;
  }

  if (clearance <= stop_clearance || clearance < 0.0) {
    response.status = "blocked";
    response.scale = 0.0;
    response.qdot = zero_qdot_like(qdot);
    return response;
  }

  double gradient_norm_sq = 0.0;
  double closing_speed = 0.0;
  const std::size_t count = std::min(clearance_gradient.size(), qdot.size());
  for (std::size_t i = 0; i < count; ++i) {
    gradient_norm_sq += clearance_gradient[i] * clearance_gradient[i];
    closing_speed += clearance_gradient[i] * qdot[i];
  }
  response.closing_speed = closing_speed;

  if (gradient_norm_sq <= 1.0e-12) {
    response.status = "clear";
    response.scale = 1.0;
    return response;
  }

  if (closing_speed >= 0.0) {
    response.status = "clear";
    response.scale = 1.0;
    return response;
  }

  if (mode == CollisionResponseMode::Scale) {
    const double scale = collision_clearance_scale(
      clearance, stop_clearance, activation_clearance);
    response.scale = scale;
    response.status = scale <= 1.0e-12 ? "blocked" : "limited";
    for (double & value : response.qdot) {
      value *= scale;
    }
    return response;
  }

  const double projection = closing_speed / gradient_norm_sq;
  for (std::size_t i = 0; i < count; ++i) {
    response.qdot[i] -= projection * clearance_gradient[i];
  }

  response.status = "limited";
  response.scale = 0.0;
  double residual_closing = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    residual_closing += clearance_gradient[i] * response.qdot[i];
  }
  const double original_norm = std::abs(closing_speed);
  if (original_norm > 1.0e-12) {
    response.scale = std::clamp(
      1.0 - std::abs(residual_closing) / original_norm, 0.0, 1.0);
  }
  return response;
}

}  // namespace mur_control
