#ifndef CAR_CONTROL__SIM__D_TASK_TRACK_GEOMETRY_HPP_
#define CAR_CONTROL__SIM__D_TASK_TRACK_GEOMETRY_HPP_

#include <cstddef>

namespace car_control::sim
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

enum class TrackSegment : std::size_t {A_TO_B = 0, B_TO_C, C_TO_D, D_TO_A, INVALID};

struct TrackNearest
{
  TrackSegment segment{TrackSegment::INVALID};
  double distance_m{0.0};
};

class DTaskTrackGeometry
{
public:
  static constexpr double kLeftX = 1.50;
  static constexpr double kRightX = 3.00;
  static constexpr double kLowerY = 2.00;
  static constexpr double kUpperY = 3.50;
  static constexpr double kCenterX = 2.25;
  static constexpr double kRadius = 0.75;

  double distance_to_centerline(const Point2 & point) const noexcept;
  double distance_to_segment(const Point2 & point, TrackSegment segment) const noexcept;
  TrackNearest nearest(const Point2 & point) const noexcept;
  static const char * segment_name(TrackSegment segment) noexcept;
};

}  // namespace car_control::sim

#endif
