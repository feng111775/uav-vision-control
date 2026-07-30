#include "car_control/sim/d_task_track_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace car_control::sim
{
namespace
{
double segment_distance(const Point2 & p, const Point2 & a, const Point2 & b) noexcept
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
}  // namespace

double DTaskTrackGeometry::distance_to_centerline(const Point2 & p) const noexcept
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
    return std::numeric_limits<double>::infinity();
  }
  const double left = segment_distance(p, {kLeftX, kLowerY}, {kLeftX, kUpperY});
  const double right = segment_distance(p, {kRightX, kLowerY}, {kRightX, kUpperY});
  const double radial = std::abs(std::hypot(p.x - kCenterX, p.y - kUpperY) - kRadius);
  const double upper = p.y >= kUpperY ? radial :
    std::min(std::hypot(p.x - kLeftX, p.y - kUpperY),
    std::hypot(p.x - kRightX, p.y - kUpperY));
  const double lower_radial = std::abs(std::hypot(p.x - kCenterX, p.y - kLowerY) - kRadius);
  const double lower = p.y <= kLowerY ? lower_radial :
    std::min(std::hypot(p.x - kLeftX, p.y - kLowerY),
    std::hypot(p.x - kRightX, p.y - kLowerY));
  return std::min({left, right, upper, lower});
}

}  // namespace car_control::sim
