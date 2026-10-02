#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace mur_control
{
// A complete, uninterrupted stationary sampling window is required on every activation.
class WrenchBias
{
public:
  using Sample = std::array<double, 6>;
  void reset()
  {
    sum_ = {};
    bias_ = {};
    count_ = 0;
    ready_ = false;
  }
  bool update(const Sample & sample, double time, double duration, bool stationary)
  {
    bool valid = stationary && std::isfinite(time);
    for (double value : sample) {valid = valid && std::isfinite(value);}
    if (!valid) {reset(); return false;}
    if (ready_) {return true;}
    if (count_ && time < last_time_) {reset();}
    if (count_ == 0) {start_ = time;}
    last_time_ = time;
    for (std::size_t i = 0; i < sample.size(); ++i) {sum_[i] += sample[i];}
    ++count_;
    if (count_ >= 2 && time - start_ >= duration) {
      for (std::size_t i = 0; i < sample.size(); ++i) {bias_[i] = sum_[i] / count_;}
      ready_ = true;
    }
    return ready_;
  }
  const Sample & value() const {return bias_;}
private:
  Sample sum_{};
  Sample bias_{};
  std::size_t count_{0};
  double start_{0.0};
  double last_time_{0.0};
  bool ready_{false};
};
}  // namespace mur_control
