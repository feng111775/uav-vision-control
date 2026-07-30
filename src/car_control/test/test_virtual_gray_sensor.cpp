#include "car_control/sim/virtual_gray_sensor.hpp"
#include "test_common.hpp"

#include <cmath>
#include <limits>
#include <vector>

int main()
{
  using car_control::sim::VirtualGraySensor;
  using car_control::sim::VirtualGraySensorConfig;
  int failures = 0;
  VirtualGraySensorConfig config;
  config.channel_positions_m = {-0.045, -0.030, -0.015, 0.0, 0.015, 0.030, 0.045};

  VirtualGraySensor centered(config);
  const auto center = centered.sample(1.50, 2.00, M_PI_2);
  failures += check(center.valid && center.line.detected, "center detects line");
  failures += check(std::abs(center.line.error) < 0.05, "center error");
  const auto car_left = centered.sample(1.47, 2.00, M_PI_2);
  const auto car_right = centered.sample(1.53, 2.00, M_PI_2);
  failures += check(car_left.line.error < 0.0, "line is sensor-right");
  failures += check(car_right.line.error > 0.0, "line is sensor-left");
  const auto lost = centered.sample(0.50, 0.50, 0.0);
  failures += check(lost.valid && !lost.line.detected, "lost line");

  const auto upper = centered.sample(2.10, 4.25, 0.0);
  const auto lower = centered.sample(2.40, 1.25, M_PI);
  failures += check(upper.line.detected, "upper semicircle");
  failures += check(lower.line.detected, "lower semicircle");

  auto inverted_config = config;
  inverted_config.black_line_is_active = false;
  VirtualGraySensor inverted(inverted_config);
  const auto inverse = inverted.sample(1.50, 2.00, M_PI_2);
  failures += check(inverse.line.detected && inverse.values[3] > 0.9, "polarity inversion");

  auto four_config = config;
  four_config.channel_positions_m = {-0.03, -0.01, 0.01, 0.03};
  VirtualGraySensor four(four_config);
  failures += check(four.sample(1.50, 2.00, M_PI_2).values.size() == 4, "arbitrary count");

  auto adjusted_config = config;
  adjusted_config.brightness_gain = 0.5;
  adjusted_config.brightness_bias = 0.25;
  VirtualGraySensor adjusted(adjusted_config);
  const auto adjusted_output = adjusted.sample(0.50, 0.50, 0.0);
  failures += check(near(adjusted_output.values[0], 0.75), "gain and bias");

  auto noisy_config = config;
  noisy_config.noise_stddev = 0.2;
  noisy_config.random_seed = 17;
  VirtualGraySensor noisy_a(noisy_config);
  VirtualGraySensor noisy_b(noisy_config);
  failures += check(
    noisy_a.sample(1.50, 2.00, M_PI_2).values ==
    noisy_b.sample(1.50, 2.00, M_PI_2).values, "reproducible noise");

  const auto bounded = noisy_a.sample(1.50, 2.00, M_PI_2);
  for (const double value : bounded.values) {
    failures += check(std::isfinite(value) && value >= 0.0 && value <= 1.0, "bounded");
  }
  failures += check(
    !centered.sample(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0).valid,
    "nan input");
  failures += check(
    !centered.sample(0.0, std::numeric_limits<double>::infinity(), 0.0).valid,
    "infinite input");
  auto invalid_config = config;
  invalid_config.sample_width_m = -1.0;
  failures += check(!VirtualGraySensor(invalid_config).valid(), "invalid configuration");
  return failures;
}
