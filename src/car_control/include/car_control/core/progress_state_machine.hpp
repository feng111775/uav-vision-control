#ifndef CAR_CONTROL__CORE__PROGRESS_STATE_MACHINE_HPP_
#define CAR_CONTROL__CORE__PROGRESS_STATE_MACHINE_HPP_

#include "car_control/core/types.hpp"

namespace car_control::core
{

struct ProgressConfig
{
  double minimum_node_distance_m{0.5};
  double node_lockout_s{0.5};
  double ab_target_s{15.0};
  double lap_target_s{90.0};
};

struct ProgressOutput
{
  ProgressState state{ProgressState::WAIT_START};
  bool marker_accepted{false};
  bool valid{false};
  double ab_elapsed_s{0.0};
  double lap_elapsed_s{0.0};
  bool ab_within_target{false};
  bool lap_within_target{false};
};

class ProgressStateMachine
{
public:
  explicit ProgressStateMachine(ProgressConfig config) noexcept;
  ProgressOutput update(
    bool start_event, Marker marker, double cumulative_distance_m,
    double competition_time_s) noexcept;
  void reset() noexcept;
  ProgressState state() const noexcept;

private:
  Marker expected_marker() const noexcept;
  bool config_valid() const noexcept;
  ProgressOutput output(bool accepted, bool valid) const noexcept;

  ProgressConfig config_;
  ProgressState state_{ProgressState::WAIT_START};
  double start_distance_m_{0.0};
  double start_time_s_{0.0};
  double last_node_distance_m_{0.0};
  double last_node_time_s_{0.0};
  double ab_elapsed_s_{0.0};
  double lap_elapsed_s_{0.0};
  bool ab_recorded_{false};
  bool lap_recorded_{false};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__PROGRESS_STATE_MACHINE_HPP_
