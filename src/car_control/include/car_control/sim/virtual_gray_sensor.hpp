#ifndef CAR_CONTROL__SIM__VIRTUAL_GRAY_SENSOR_HPP_
#define CAR_CONTROL__SIM__VIRTUAL_GRAY_SENSOR_HPP_

#include "car_control/core/line_error_estimator.hpp"
#include "car_control/sim/d_task_track_geometry.hpp"

#include <cstdint>
#include <random>
#include <vector>

namespace car_control::sim
{

struct VirtualGraySensorConfig
{
  double sensor_forward_offset_m{0.20};
  std::vector<double> channel_positions_m;
  double line_width_m{0.020};
  double sample_width_m{0.008};
  bool black_line_is_active{true};
  double minimum_activation{0.2};
  double brightness_gain{1.0};
  double brightness_bias{0.0};
  double noise_stddev{0.0};
  std::uint32_t random_seed{2026};
};

struct VirtualGraySensorOutput
{
  std::vector<double> values;
  core::LineError line;
  bool valid{false};
};

class VirtualGraySensor
{
public:
  explicit VirtualGraySensor(const VirtualGraySensorConfig & config);
  bool valid() const noexcept;
  VirtualGraySensorOutput sample(double world_x, double world_y, double yaw) noexcept;

private:
  VirtualGraySensorConfig config_;
  DTaskTrackGeometry track_;
  std::mt19937 generator_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  bool valid_{false};
};

}  // namespace car_control::sim

#endif
