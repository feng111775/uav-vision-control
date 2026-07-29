#ifndef CAR_CONTROL__CORE__ENCODER_VELOCITY_ESTIMATOR_HPP_
#define CAR_CONTROL__CORE__ENCODER_VELOCITY_ESTIMATOR_HPP_

#include <cstdint>

namespace car_control::core
{

struct EncoderVelocity
{
  double angular_rad_s{0.0};
  double linear_m_s{0.0};
  bool valid{false};
};

EncoderVelocity estimate_encoder_velocity(
  std::int64_t delta_counts, double dt_s, double counts_per_revolution,
  double wheel_radius_m) noexcept;

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__ENCODER_VELOCITY_ESTIMATOR_HPP_
