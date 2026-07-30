#include "car_control/sim/virtual_gray_sensor.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::sim
{
namespace
{
bool finite_config(const VirtualGraySensorConfig & c) noexcept
{
  if (c.channel_positions_m.empty() || !std::isfinite(c.sensor_forward_offset_m) ||
    !std::isfinite(c.line_width_m) || c.line_width_m <= 0.0 ||
    !std::isfinite(c.sample_width_m) || c.sample_width_m <= 0.0 ||
    !std::isfinite(c.minimum_activation) || c.minimum_activation < 0.0 ||
    !std::isfinite(c.brightness_gain) || !std::isfinite(c.brightness_bias) ||
    !std::isfinite(c.noise_stddev) || c.noise_stddev < 0.0)
  {
    return false;
  }
  return std::all_of(c.channel_positions_m.begin(), c.channel_positions_m.end(),
    [](double value) {return std::isfinite(value);});
}

double smooth_coverage(double distance, double half_line) noexcept
{
  constexpr double edge_m = 0.001;
  const double t = std::clamp((half_line + edge_m - distance) / (2.0 * edge_m), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}
}  // namespace

VirtualGraySensor::VirtualGraySensor(const VirtualGraySensorConfig & config)
: config_(config), generator_(config.random_seed), valid_(finite_config(config))
{
}

bool VirtualGraySensor::valid() const noexcept {return valid_;}

VirtualGraySensorOutput VirtualGraySensor::sample(
  double world_x, double world_y, double yaw) noexcept
{
  VirtualGraySensorOutput output;
  if (!valid_ || !std::isfinite(world_x) || !std::isfinite(world_y) || !std::isfinite(yaw)) {
    return output;
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  output.values.reserve(config_.channel_positions_m.size());
  constexpr int samples = 9;
  for (const double lateral : config_.channel_positions_m) {
    double coverage = 0.0;
    for (int index = 0; index < samples; ++index) {
      const double fraction = (static_cast<double>(index) + 0.5) / samples - 0.5;
      const double local_y = lateral + fraction * config_.sample_width_m;
      const Point2 point{
        world_x + c * config_.sensor_forward_offset_m - s * local_y,
        world_y + s * config_.sensor_forward_offset_m + c * local_y};
      coverage += smooth_coverage(
        track_.distance_to_centerline(point), config_.line_width_m * 0.5);
    }
    coverage /= samples;
    double value = config_.black_line_is_active ? 1.0 - coverage : coverage;
    value = config_.brightness_gain * value + config_.brightness_bias;
    if (config_.noise_stddev > 0.0) {
      value += config_.noise_stddev * normal_(generator_);
    }
    output.values.push_back(std::clamp(value, 0.0, 1.0));
  }
  output.line = core::estimate_line_error(
    output.values, config_.channel_positions_m, config_.black_line_is_active,
    config_.minimum_activation);
  output.valid = output.line.valid &&
    std::all_of(output.values.begin(), output.values.end(), [](double v) {return std::isfinite(v);});
  return output;
}

}  // namespace car_control::sim
