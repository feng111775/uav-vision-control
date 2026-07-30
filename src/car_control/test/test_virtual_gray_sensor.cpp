#include "car_control/sim/virtual_gray_sensor.hpp"
#include "car_control/sim/d_task_track_geometry.hpp"
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
  car_control::sim::DTaskTrackGeometry track;
  using car_control::sim::Point2;
  using car_control::sim::TrackSegment;
  failures += check(
    near(track.distance_to_segment({1.50, 2.75}, TrackSegment::A_TO_B), 0.0),
    "A-B finite segment");
  failures += check(
    near(track.distance_to_segment({2.25, 4.25}, TrackSegment::B_TO_C), 0.0),
    "upper semicircle geometry");
  failures += check(
    near(track.distance_to_segment({3.00, 2.75}, TrackSegment::C_TO_D), 0.0),
    "C-D right straight geometry");
  failures += check(
    near(track.distance_to_segment({2.25, 1.25}, TrackSegment::D_TO_A), 0.0),
    "lower semicircle geometry");
  failures += check(
    track.distance_to_segment({2.25, 2.75}, TrackSegment::B_TO_C) > 0.5,
    "upper arc cannot extend into lower half");
  failures += check(
    track.distance_to_segment({2.25, 2.75}, TrackSegment::D_TO_A) > 0.5,
    "lower arc cannot extend into upper half");
  for (const auto connection : std::vector<std::pair<Point2, std::pair<TrackSegment, TrackSegment>>>{
      {{1.50, 3.50}, {TrackSegment::A_TO_B, TrackSegment::B_TO_C}},
      {{3.00, 3.50}, {TrackSegment::B_TO_C, TrackSegment::C_TO_D}},
      {{3.00, 2.00}, {TrackSegment::C_TO_D, TrackSegment::D_TO_A}},
      {{1.50, 2.00}, {TrackSegment::D_TO_A, TrackSegment::A_TO_B}}})
  {
    failures += check(
      near(track.distance_to_segment(connection.first, connection.second.first), 0.0) &&
      near(track.distance_to_segment(connection.first, connection.second.second), 0.0),
      "track connection is continuous");
  }
  const auto center = centered.sample(1.50, 1.80, M_PI_2);
  failures += check(center.valid && center.line.detected, "center detects line");
  failures += check(std::abs(center.line.error) < 0.05, "center error");
  const auto car_left = centered.sample(1.47, 1.80, M_PI_2);
  const auto car_right = centered.sample(1.53, 1.80, M_PI_2);
  failures += check(car_left.line.error < 0.0, "line is sensor-right");
  failures += check(car_right.line.error > 0.0, "line is sensor-left");
  const auto lost = centered.sample(0.50, 0.50, 0.0);
  failures += check(lost.valid && !lost.line.detected, "lost line");
  failures += check(
    centered.sample(2.80, 2.75, 0.0).line.detected,
    "sensor transform yaw zero");
  failures += check(
    centered.sample(1.50, 1.80, M_PI_2).line.detected,
    "sensor transform yaw pi/2");
  failures += check(
    centered.sample(1.70, 2.75, M_PI).line.detected,
    "sensor transform yaw pi");
  failures += check(
    centered.sample(3.00, 3.70, -M_PI_2).line.detected,
    "sensor transform yaw minus pi/2");

  const auto upper = centered.sample(2.05, 4.25, 0.0);
  const auto lower = centered.sample(2.45, 1.25, M_PI);
  failures += check(upper.line.detected, "upper semicircle");
  failures += check(lower.line.detected, "lower semicircle");

  auto inverted_config = config;
  inverted_config.black_line_is_active = false;
  VirtualGraySensor inverted(inverted_config);
  const auto inverse = inverted.sample(1.50, 1.80, M_PI_2);
  failures += check(inverse.line.detected && inverse.values[3] > 0.9, "polarity inversion");

  auto four_config = config;
  four_config.channel_positions_m = {-0.03, -0.01, 0.01, 0.03};
  VirtualGraySensor four(four_config);
  failures += check(four.sample(1.50, 1.80, M_PI_2).values.size() == 4, "arbitrary count");

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
    noisy_a.sample(1.50, 1.80, M_PI_2).values ==
    noisy_b.sample(1.50, 1.80, M_PI_2).values, "reproducible noise");

  const auto bounded = noisy_a.sample(1.50, 1.80, M_PI_2);
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
