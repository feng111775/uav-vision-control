#ifndef CAR_CONTROL__SIM__PROGRESS_DETECTOR_HPP_
#define CAR_CONTROL__SIM__PROGRESS_DETECTOR_HPP_

#include "car_control/core/types.hpp"

namespace car_control::sim
{

struct WorldPoint
{
  double x{0.0};
  double y{0.0};
};

struct ProgressDetectorConfig
{
  double initial_base_x{1.50};
  double initial_base_y{1.80};
  double initial_yaw{1.5707963267948966};
  double node_radius_m{0.25};
  WorldPoint a{1.50, 2.00};
  WorldPoint b{1.50, 3.50};
  WorldPoint c{3.00, 3.50};
  WorldPoint d{3.00, 2.00};
};

class ProgressDetector
{
public:
  explicit ProgressDetector(ProgressDetectorConfig config) noexcept;
  bool valid() const noexcept;
  WorldPoint odom_to_world(double odom_x, double odom_y) const noexcept;
  core::Marker update(
    double odom_x, double odom_y, core::ProgressState state) noexcept;
  core::Marker update_world(
    double world_x, double world_y, core::ProgressState state) noexcept;
  void reset() noexcept;

private:
  core::Marker expected(core::ProgressState state) const noexcept;
  const WorldPoint & point(core::Marker marker) const noexcept;
  ProgressDetectorConfig config_;
  core::Marker inside_{core::Marker::NONE};
  bool valid_{false};
};

}  // namespace car_control::sim

#endif
