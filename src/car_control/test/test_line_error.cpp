#include "car_control/core/line_error_estimator.hpp"
#include "test_common.hpp"

#include <vector>

int main()
{
  using car_control::core::estimate_line_error;
  const std::vector<double> positions{-1.0, 0.0, 1.0};
  int failures = 0;
  const auto center = estimate_line_error({1.0, 0.0, 1.0}, positions, true, 0.2);
  failures += check(center.valid && center.detected && near(center.error, 0.0), "center");
  failures += check(estimate_line_error({0.0, 1.0, 1.0}, positions, true, 0.2).error < 0.0, "left");
  failures += check(estimate_line_error({1.0, 1.0, 0.0}, positions, true, 0.2).error > 0.0, "right");
  const auto missing = estimate_line_error({1.0, 1.0, 1.0}, positions, true, 0.2);
  failures += check(missing.valid && !missing.detected && near(missing.total_activation, 0.0), "no line");
  const auto inverted = estimate_line_error({0.0, 1.0, 0.0}, positions, false, 0.2);
  failures += check(inverted.valid && inverted.detected && near(inverted.error, 0.0), "polarity");
  failures += check(!estimate_line_error({0.0}, positions, true, 0.2).valid, "size mismatch");
  return failures;
}
