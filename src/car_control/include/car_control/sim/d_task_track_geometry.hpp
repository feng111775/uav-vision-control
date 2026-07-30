#ifndef CAR_CONTROL__SIM__D_TASK_TRACK_GEOMETRY_HPP_
#define CAR_CONTROL__SIM__D_TASK_TRACK_GEOMETRY_HPP_

namespace car_control::sim
{

struct Point2
{
  double x{0.0};
  double y{0.0};
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
};

}  // namespace car_control::sim

#endif
