#include "car_control/core/line_recovery.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::LineRecovery;
  using car_control::core::LineState;
  int failures = 0;
  LineRecovery recovery({0.5, 0.3});
  failures += check(recovery.update(true, -0.4, 0.1).state == LineState::TRACKING, "tracking");
  const auto lost = recovery.update(false, 0.0, 0.1);
  failures += check(lost.state == LineState::RECOVERY && lost.suggested_steering < 0.0,
    "recovery retains direction");
  failures += check(recovery.update(true, 0.2, 0.1).state == LineState::TRACKING, "reacquire");
  recovery.update(false, 0.0, 0.3);
  failures += check(recovery.update(false, 0.0, 0.3).state == LineState::FAULT, "timeout fault");
  recovery.reset();
  failures += check(recovery.update(false, 0.0, 0.1).state == LineState::RECOVERY, "reset");
  failures += check(!recovery.update(false, 0.0, 0.0).valid, "invalid dt");
  return failures;
}
