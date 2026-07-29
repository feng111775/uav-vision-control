#include "car_control/core/encoder_velocity_estimator.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::estimate_encoder_velocity;
  int failures = 0;
  const auto zero = estimate_encoder_velocity(0, 0.1, 100.0, 0.05);
  failures += check(zero.valid && near(zero.linear_m_s, 0.0), "zero counts");
  const auto positive = estimate_encoder_velocity(50, 0.5, 100.0, 0.1);
  failures += check(
    positive.valid && near(positive.angular_rad_s, 6.283185307179586),
    "positive angular velocity");
  failures += check(near(positive.linear_m_s, 0.6283185307179586), "linear velocity");
  const auto negative = estimate_encoder_velocity(-50, 0.5, 100.0, 0.1);
  failures += check(negative.valid && negative.linear_m_s < 0.0, "reverse velocity");
  failures += check(!estimate_encoder_velocity(1, 0.0, 100.0, 0.1).valid, "invalid dt");
  failures += check(!estimate_encoder_velocity(1, 0.1, 0.0, 0.1).valid, "invalid CPR");
  failures += check(!estimate_encoder_velocity(1, 0.1, 100.0, 0.0).valid, "invalid radius");
  return failures;
}
