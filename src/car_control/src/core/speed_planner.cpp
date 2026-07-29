#include "car_control/core/speed_planner.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::core
{

WheelTargets plan_wheel_speeds(
  const SpeedPlannerConfig & config, double base_speed_m_s, double line_error,
  double steering_correction_m_s, LineState state) noexcept
{
  const bool config_valid =
    config.maximum_speed_m_s > 0.0 &&
    config.minimum_tracking_speed_m_s >= 0.0 &&
    config.minimum_tracking_speed_m_s <= config.maximum_speed_m_s &&
    config.error_slowdown_gain_m_s >= 0.0 &&
    config.maximum_wheel_speed_m_s > 0.0 &&
    config.recovery_speed_m_s > 0.0;
  if (!config_valid || !std::isfinite(base_speed_m_s) ||
    !std::isfinite(line_error) || !std::isfinite(steering_correction_m_s))
  {
    return {};
  }
  if (state == LineState::FAULT) {
    return {0.0, 0.0, true};
  }
  double forward = config.recovery_speed_m_s;
  if (state == LineState::TRACKING) {
    const double requested = std::clamp(
      base_speed_m_s, config.minimum_tracking_speed_m_s,
      config.maximum_speed_m_s);
    forward = std::max(
      config.minimum_tracking_speed_m_s,
      requested - config.error_slowdown_gain_m_s * std::abs(line_error));
  }
  double left = forward - steering_correction_m_s;
  double right = forward + steering_correction_m_s;
  const double peak = std::max(std::abs(left), std::abs(right));
  if (peak > config.maximum_wheel_speed_m_s) {
    const double scale = config.maximum_wheel_speed_m_s / peak;
    left *= scale;
    right *= scale;
  }
  return {left, right, true};
}

}  // namespace car_control::core
