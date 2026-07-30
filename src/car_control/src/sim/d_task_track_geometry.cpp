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
  return nearest(p).distance_m;
}

double DTaskTrackGeometry::distance_to_segment(
  const Point2 & p, TrackSegment segment) const noexcept
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
    return std::numeric_limits<double>::infinity();
  }
  switch (segment) {
    case TrackSegment::A_TO_B:
      return segment_distance(p, {kLeftX, kLowerY}, {kLeftX, kUpperY});
    case TrackSegment::C_TO_D:
      return segment_distance(p, {kRightX, kUpperY}, {kRightX, kLowerY});
    case TrackSegment::B_TO_C:
      if (p.y >= kUpperY) {
        return std::abs(std::hypot(p.x - kCenterX, p.y - kUpperY) - kRadius);
      }
      return std::min(
        std::hypot(p.x - kLeftX, p.y - kUpperY),
        std::hypot(p.x - kRightX, p.y - kUpperY));
    case TrackSegment::D_TO_A:
      if (p.y <= kLowerY) {
        return std::abs(std::hypot(p.x - kCenterX, p.y - kLowerY) - kRadius);
      }
      return std::min(
        std::hypot(p.x - kLeftX, p.y - kLowerY),
        std::hypot(p.x - kRightX, p.y - kLowerY));
    default:
      return std::numeric_limits<double>::infinity();
  }
}

TrackNearest DTaskTrackGeometry::nearest(const Point2 & p) const noexcept
{
  TrackNearest result;
  result.distance_m = std::numeric_limits<double>::infinity();
  for (const auto segment :
    {TrackSegment::A_TO_B, TrackSegment::B_TO_C, TrackSegment::C_TO_D, TrackSegment::D_TO_A})
  {
    const double distance = distance_to_segment(p, segment);
    if (distance < result.distance_m) {
      result = {segment, distance};
    }
  }
  return result;
}

const char * DTaskTrackGeometry::segment_name(TrackSegment segment) noexcept
{
  switch (segment) {
    case TrackSegment::A_TO_B: return "A_TO_B";
    case TrackSegment::B_TO_C: return "B_TO_C";
    case TrackSegment::C_TO_D: return "C_TO_D";
    case TrackSegment::D_TO_A: return "D_TO_A";
    default: return "INVALID";
  }
}

}  // namespace car_control::sim
