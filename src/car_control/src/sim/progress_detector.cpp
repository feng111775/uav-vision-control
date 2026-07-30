#include "car_control/sim/progress_detector.hpp"

#include <cmath>
#include <initializer_list>

namespace car_control::sim
{
namespace
{
bool finite_point(const WorldPoint & p) noexcept
{
  return std::isfinite(p.x) && std::isfinite(p.y);
}
}  // namespace

ProgressDetector::ProgressDetector(ProgressDetectorConfig config) noexcept
: config_(config)
{
  valid_ = std::isfinite(config_.initial_base_x) &&
    std::isfinite(config_.initial_base_y) && std::isfinite(config_.initial_yaw) &&
    std::isfinite(config_.node_radius_m) && config_.node_radius_m > 0.0 &&
    finite_point(config_.a) && finite_point(config_.b) &&
    finite_point(config_.c) && finite_point(config_.d);
}

bool ProgressDetector::valid() const noexcept {return valid_;}

WorldPoint ProgressDetector::odom_to_world(double odom_x, double odom_y) const noexcept
{
  if (!valid_ || !std::isfinite(odom_x) || !std::isfinite(odom_y)) {
    return {NAN, NAN};
  }
  const double c = std::cos(config_.initial_yaw);
  const double s = std::sin(config_.initial_yaw);
  return {
    config_.initial_base_x + c * odom_x - s * odom_y,
    config_.initial_base_y + s * odom_x + c * odom_y};
}

core::Marker ProgressDetector::expected(core::ProgressState state) const noexcept
{
  switch (state) {
    case core::ProgressState::A_TO_B: return core::Marker::B;
    case core::ProgressState::B_TO_C: return core::Marker::C;
    case core::ProgressState::C_TO_D: return core::Marker::D;
    case core::ProgressState::D_TO_A: return core::Marker::A;
    default: return core::Marker::NONE;
  }
}

const WorldPoint & ProgressDetector::point(core::Marker marker) const noexcept
{
  switch (marker) {
    case core::Marker::A: return config_.a;
    case core::Marker::B: return config_.b;
    case core::Marker::C: return config_.c;
    default: return config_.d;
  }
}

core::Marker ProgressDetector::update(
  double odom_x, double odom_y, core::ProgressState state) noexcept
{
  const WorldPoint world = odom_to_world(odom_x, odom_y);
  return update_world(world.x, world.y, state);
}

core::Marker ProgressDetector::update_world(
  double world_x, double world_y, core::ProgressState state) noexcept
{
  const WorldPoint world{world_x, world_y};
  if (!std::isfinite(world.x) || !std::isfinite(world.y)) {
    return core::Marker::NONE;
  }
  core::Marker current = core::Marker::NONE;
  for (const core::Marker marker :
    {core::Marker::A, core::Marker::B, core::Marker::C, core::Marker::D})
  {
    const auto & target = point(marker);
    if (std::hypot(world.x - target.x, world.y - target.y) <= config_.node_radius_m) {
      current = marker;
      break;
    }
  }
  const bool entered = current != core::Marker::NONE && current != inside_;
  inside_ = current;
  return entered && current == expected(state) ? current : core::Marker::NONE;
}

void ProgressDetector::reset() noexcept {inside_ = core::Marker::NONE;}

}  // namespace car_control::sim
