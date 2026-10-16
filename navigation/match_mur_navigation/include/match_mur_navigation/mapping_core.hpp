#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include <tf2/LinearMath/Transform.hpp>

namespace mur_mapping {

struct TimedPose { double time; tf2::Transform pose; };

class PoseBuffer {
 public:
  explicit PoseBuffer(double max_gap = 0.1) : max_gap_(max_gap) {}
  void add(double time, const tf2::Transform &pose) {
    // Ignore reordered duplicates; never interpolate across a backwards clock jump.
    if (!poses_.empty() && time <= poses_.back().time) return;
    poses_.push_back({time, pose});
    while (!poses_.empty() && poses_.front().time < time - 10.0) poses_.pop_front();
  }
  bool at(double time, tf2::Transform &result) const {
    auto hi = std::lower_bound(poses_.begin(), poses_.end(), time,
      [](const TimedPose &p, double t) { return p.time < t; });
    if (hi == poses_.end()) return false;
    if (std::abs(hi->time - time) < 1e-8) { result = hi->pose; return true; }
    if (hi == poses_.begin()) return false;
    const auto &lo = *std::prev(hi);
    if (hi->time - lo.time > max_gap_) return false;
    double f = (time - lo.time) / (hi->time - lo.time);
    result = tf2::Transform(lo.pose.getRotation().slerp(hi->pose.getRotation(), f),
      lo.pose.getOrigin().lerp(hi->pose.getOrigin(), f));
    return true;
  }
  double latest() const { return poses_.empty() ? -1.0 : poses_.back().time; }
  void clear() { poses_.clear(); }
 private:
  std::deque<TimedPose> poses_;
  double max_gap_;
};

struct Ray { double x0, y0, x1, y1; bool hit; };

class Grid {
 public:
  Grid(double resolution, size_t max_cells) : resolution_(resolution), max_cells_(max_cells) {
    if (!std::isfinite(resolution) || resolution <= 0 || max_cells == 0)
      throw std::invalid_argument("Invalid grid limits");
  }
  bool integrate(const std::vector<Ray> &rays) {
    if (rays.empty()) return true;
    for (const auto &r : rays)
      for (double v : {r.x0, r.y0, r.x1, r.y1})
        if (!std::isfinite(v) || std::abs(v / resolution_) > 1e8) return false;
    int lo_x = width_ ? x_ : coord(rays.front().x0);
    int lo_y = height_ ? y_ : coord(rays.front().y0);
    int hi_x = width_ ? x_ + width_ - 1 : lo_x;
    int hi_y = height_ ? y_ + height_ - 1 : lo_y;
    for (const auto &r : rays) {
      lo_x = std::min({lo_x, coord(r.x0), coord(r.x1)});
      hi_x = std::max({hi_x, coord(r.x0), coord(r.x1)});
      lo_y = std::min({lo_y, coord(r.y0), coord(r.y1)});
      hi_y = std::max({hi_y, coord(r.y0), coord(r.y1)});
    }
    // Grow in blocks to avoid reallocating the entire grid for every cell.
    lo_x = static_cast<int>(std::floor(lo_x / 64.0)) * 64;
    lo_y = static_cast<int>(std::floor(lo_y / 64.0)) * 64;
    hi_x = static_cast<int>(std::floor(hi_x / 64.0) + 1) * 64;
    hi_y = static_cast<int>(std::floor(hi_y / 64.0) + 1) * 64;
    const int w = hi_x - lo_x, h = hi_y - lo_y;
    if (static_cast<uint64_t>(w) * h > max_cells_) return false;
    if (w != width_ || h != height_ || lo_x != x_ || lo_y != y_) {
      std::vector<float> next(static_cast<size_t>(w) * h, std::numeric_limits<float>::quiet_NaN());
      for (int j = 0; j < height_; ++j)
        std::copy_n(cells_.begin() + static_cast<size_t>(j) * width_, width_,
          next.begin() + static_cast<size_t>(j + y_ - lo_y) * w + x_ - lo_x);
      cells_.swap(next); x_ = lo_x; y_ = lo_y; width_ = w; height_ = h;
    }
    std::unordered_set<size_t> free, occupied;
    for (const auto &r : rays) trace(r, free, occupied);
    // Correlated beams count once per scan; an observed endpoint takes precedence.
    for (size_t index : free) if (!occupied.count(index)) update(index, false);
    for (size_t index : occupied) update(index, true);
    return true;
  }
  int width() const { return width_; }
  int height() const { return height_; }
  double origin_x() const { return x_ * resolution_; }
  double origin_y() const { return y_ * resolution_; }
  double resolution() const { return resolution_; }
  std::vector<int8_t> data() const {
    std::vector<int8_t> out; out.reserve(cells_.size());
    for (float v : cells_)
      out.push_back(std::isnan(v) ? -1 : static_cast<int8_t>(std::lround(100.0 / (1.0 + std::exp(-v)))));
    return out;
  }
 private:
  int coord(double v) const { return static_cast<int>(std::floor(v / resolution_)); }
  void update(size_t index, bool hit) {
    float &v = cells_[index];
    if (std::isnan(v)) v = 0;
    v = std::clamp(v + (hit ? 0.85f : -0.4f), -2.0f, 3.5f);
  }
  void trace(const Ray &r, std::unordered_set<size_t> &free, std::unordered_set<size_t> &occupied) {
    int x = coord(r.x0), y = coord(r.y0), end_x = coord(r.x1), end_y = coord(r.y1);
    int dx = std::abs(end_x - x), sx = x < end_x ? 1 : -1;
    int dy = -std::abs(end_y - y), sy = y < end_y ? 1 : -1, err = dx + dy;
    while (true) {
      const bool end = x == end_x && y == end_y;
      size_t index = static_cast<size_t>(y - y_) * width_ + x - x_;
      if (end && r.hit) occupied.insert(index);
      else free.insert(index);
      if (end) break;
      const int twice = 2 * err;
      if (twice >= dy) { err += dy; x += sx; }
      if (twice <= dx) { err += dx; y += sy; }
    }
  }
  double resolution_;
  size_t max_cells_;
  int x_ = 0, y_ = 0, width_ = 0, height_ = 0;
  std::vector<float> cells_;
};
}  // namespace mur_mapping
