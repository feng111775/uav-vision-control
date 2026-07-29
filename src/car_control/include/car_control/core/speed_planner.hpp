#ifndef CAR_CONTROL__CORE__SPEED_PLANNER_HPP_
#define CAR_CONTROL__CORE__SPEED_PLANNER_HPP_

#include "car_control/core/types.hpp"

namespace car_control::core
{

struct SpeedPlannerConfig
{
  double maximum_speed_m_s{1.0};
  double minimum_tracking_speed_m_s{0.2};
  double error_slowdown_gain_m_s{0.3};
  double maximum_wheel_speed_m_s{1.2};
  double recovery_speed_m_s{0.15};
};

struct WheelTargets
{
  double left_m_s{0.0};
  double right_m_s{0.0};
  bool valid{false};
};

WheelTargets plan_wheel_speeds(
  const SpeedPlannerConfig & config, double base_speed_m_s, double line_error,
  double steering_correction_m_s, LineState state) noexcept;

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__SPEED_PLANNER_HPP_
