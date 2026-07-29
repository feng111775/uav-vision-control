#include "car_control/core/encoder_velocity_estimator.hpp"

#include <cmath>

namespace car_control::core
{

EncoderVelocity estimate_encoder_velocity(
  std::int64_t delta_counts, double dt_s, double counts_per_revolution,
  double wheel_radius_m) noexcept
{
  if (!std::isfinite(dt_s) || !std::isfinite(counts_per_revolution) ||
    !std::isfinite(wheel_radius_m) || dt_s <= 0.0 ||
    counts_per_revolution <= 0.0 || wheel_radius_m <= 0.0)
  {
    return {};
  }
  constexpr double two_pi = 6.28318530717958647692;
  const double angular =
    static_cast<double>(delta_counts) * two_pi / counts_per_revolution / dt_s;
  return {angular, angular * wheel_radius_m, true};
}

}  // namespace car_control::core
