#include "car_control/core/progress_state_machine.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::Marker;
  using car_control::core::ProgressState;
  using car_control::core::ProgressStateMachine;
  int failures = 0;
  ProgressStateMachine machine({1.0, 0.5, 15.0, 90.0});
  failures += check(machine.state() == ProgressState::WAIT_START, "initial wait");
  machine.update(true, Marker::NONE, 0.0, 0.0);
  failures += check(machine.state() == ProgressState::A_TO_B, "start to A-B");
  failures += check(!machine.update(false, Marker::C, 2.0, 2.0).marker_accepted, "wrong order");
  failures += check(!machine.update(false, Marker::B, 0.5, 2.0).marker_accepted, "distance gate");
  const auto b = machine.update(false, Marker::B, 2.0, 14.0);
  failures += check(b.marker_accepted && b.state == ProgressState::B_TO_C &&
    b.ab_within_target && near(b.ab_elapsed_s, 14.0), "B and A-B metric");
  failures += check(!machine.update(false, Marker::B, 3.0, 15.0).marker_accepted, "repeat rejected");
  failures += check(!machine.update(false, Marker::C, 2.5, 14.2).marker_accepted, "lockout");
  failures += check(machine.update(false, Marker::C, 4.0, 30.0).marker_accepted, "C accepted");
  failures += check(machine.update(false, Marker::D, 6.0, 50.0).marker_accepted, "D accepted");
  const auto finish = machine.update(false, Marker::A, 8.0, 80.0);
  failures += check(finish.marker_accepted && finish.state == ProgressState::FINISHED,
    "A finishes");
  failures += check(finish.lap_within_target && near(finish.lap_elapsed_s, 80.0), "lap metric");
  failures += check(machine.state() == ProgressState::FINISHED, "explicit finished");
  machine.reset();
  failures += check(machine.state() == ProgressState::WAIT_START, "reset");

  ProgressStateMachine slow({0.0, 0.0, 15.0, 90.0});
  slow.update(true, Marker::NONE, 0.0, 0.0);
  failures += check(!slow.update(false, Marker::B, 1.0, 15.0).ab_within_target, "strict 15s");
  slow.update(false, Marker::C, 2.0, 30.0);
  slow.update(false, Marker::D, 3.0, 60.0);
  failures += check(!slow.update(false, Marker::A, 4.0, 90.0).lap_within_target, "strict 90s");
  return failures;
}
