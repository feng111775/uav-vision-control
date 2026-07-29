#include "car_control/core/line_recovery.hpp"

#include <cmath>

namespace car_control::core
{

LineRecovery::LineRecovery(RecoveryConfig config) noexcept
: config_(config) {}

RecoveryOutput LineRecovery::update(
  bool line_detected, double line_error, double dt_s) noexcept
{
  const bool valid = config_.fault_timeout_s > 0.0 &&
    config_.search_steering > 0.0 && std::isfinite(line_error) &&
    std::isfinite(dt_s) && dt_s > 0.0;
  if (!valid) {
    return {state_, 0.0, false};
  }
  if (line_detected) {
    if (std::abs(line_error) > 1.0e-9) {
      last_direction_ = line_error < 0.0 ? -1.0 : 1.0;
    }
    state_ = LineState::TRACKING;
    lost_time_s_ = 0.0;
    return {state_, 0.0, true};
  }
  lost_time_s_ += dt_s;
  state_ = lost_time_s_ > config_.fault_timeout_s ?
    LineState::FAULT : LineState::RECOVERY;
  const double steering = state_ == LineState::RECOVERY ?
    last_direction_ * config_.search_steering : 0.0;
  return {state_, steering, true};
}

void LineRecovery::reset() noexcept
{
  state_ = LineState::TRACKING;
  lost_time_s_ = 0.0;
  last_direction_ = 1.0;
}

}  // namespace car_control::core
