#include "car_control/core/speed_planner.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <cmath>

int main()
{
  using car_control::core::LineState;
  using car_control::core::SpeedPlannerConfig;
  using car_control::core::plan_wheel_speeds;
  const SpeedPlannerConfig config{1.0, 0.2, 0.5, 1.2, 0.15};
  int failures = 0;
  const auto straight = plan_wheel_speeds(config, 1.0, 0.0, 0.0, LineState::TRACKING);
  failures += check(straight.valid && near(straight.left_m_s, 1.0) &&
    near(straight.right_m_s, 1.0), "straight high speed");
  const auto curve = plan_wheel_speeds(config, 1.0, 0.8, 0.2, LineState::TRACKING);
  failures += check(curve.left_m_s < curve.right_m_s, "differential steering");
  failures += check((curve.left_m_s + curve.right_m_s) / 2.0 < 1.0, "curve slowdown");
  const auto limited = plan_wheel_speeds(config, 1.0, 0.0, 2.0, LineState::TRACKING);
  failures += check(std::max(std::abs(limited.left_m_s), std::abs(limited.right_m_s)) <= 1.2,
    "wheel speed limit");
  failures += check(limited.left_m_s / limited.right_m_s < 0.0, "turn ratio retained");
  const auto recovery = plan_wheel_speeds(config, 1.0, 1.0, 0.1, LineState::RECOVERY);
  failures += check(recovery.valid && recovery.left_m_s != 0.0 &&
    recovery.right_m_s != 0.0, "recovery searches without ordinary stop");
  const auto fault = plan_wheel_speeds(config, 1.0, 0.0, 0.0, LineState::FAULT);
  failures += check(near(fault.left_m_s, 0.0) && near(fault.right_m_s, 0.0), "fault stop");
  return failures;
}
