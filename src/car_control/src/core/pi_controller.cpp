#include "car_control/core/pi_controller.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::core
{

PIController::PIController(PIConfig config) noexcept
: config_(config) {}

bool PIController::config_valid() const noexcept
{
  return std::isfinite(config_.kp) && std::isfinite(config_.ki) &&
         config_.ki >= 0.0 && config_.output_min < config_.output_max &&
         config_.integral_min <= config_.integral_max;
}

ControlOutput PIController::update(
  double target_m_s, double actual_m_s, double dt_s) noexcept
{
  if (!config_valid() || !std::isfinite(target_m_s) ||
    !std::isfinite(actual_m_s) || !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    return {};
  }
  const double error = target_m_s - actual_m_s;
  const double candidate = std::clamp(
    integral_ + error * dt_s, config_.integral_min, config_.integral_max);
  const double candidate_output = config_.kp * error + config_.ki * candidate;
  const bool drives_high = candidate_output > config_.output_max && error > 0.0;
  const bool drives_low = candidate_output < config_.output_min && error < 0.0;
  if (!drives_high && !drives_low) {
    integral_ = candidate;
  }
  const double output = std::clamp(
    config_.kp * error + config_.ki * integral_,
    config_.output_min, config_.output_max);
  return {output, true};
}

void PIController::reset() noexcept
{
  integral_ = 0.0;
}

double PIController::integral() const noexcept
{
  return integral_;
}

}  // namespace car_control::core
