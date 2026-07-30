#include "car_control/core/line_pd_controller.hpp"
#include "car_control/core/line_recovery.hpp"
#include "car_control/core/progress_state_machine.hpp"
#include "car_control/core/speed_planner.hpp"
#include "car_control/sim/control_safety.hpp"
#include "car_control/sim/final_lap_metrics.hpp"
#include "car_control/sim/progress_detector.hpp"
#include "test_common.hpp"

#include <cmath>
#include <limits>

int main()
{
  using namespace car_control;
  int failures = 0;
  core::LinePDController pd({0.1, 0.0, 0.2, 0.0});
  core::SpeedPlannerConfig speed{0.4, 0.1, 0.05, 0.5, 0.1};
  const auto straight_correction = pd.update(0.0, 0.02);
  auto wheels = core::plan_wheel_speeds(
    speed, 0.25, 0.0, straight_correction.value, core::LineState::TRACKING);
  auto command = sim::wheel_targets_to_chassis(wheels.left_m_s, wheels.right_m_s, 0.32);
  failures += check(command.valid && near(command.angular_z, 0.0), "center drives straight");

  pd.reset();
  const auto left = pd.update(0.5, 0.02);
  wheels = core::plan_wheel_speeds(
    speed, 0.25, 0.5, left.value, core::LineState::TRACKING);
  command = sim::wheel_targets_to_chassis(wheels.left_m_s, wheels.right_m_s, 0.32);
  failures += check(command.angular_z > 0.0, "line left commands left turn");
  pd.reset();
  const auto right = pd.update(-0.5, 0.02);
  wheels = core::plan_wheel_speeds(
    speed, 0.25, -0.5, right.value, core::LineState::TRACKING);
  command = sim::wheel_targets_to_chassis(wheels.left_m_s, wheels.right_m_s, 0.32);
  failures += check(command.angular_z < 0.0, "line right commands right turn");
  failures += check(
    sim::wheel_targets_to_chassis(0.2, 0.3, 0.32).angular_z > 0.0,
    "right wheel faster is positive yaw");
  failures += check(
    sim::wheel_targets_to_chassis(0.3, 0.2, 0.32).angular_z < 0.0,
    "left wheel faster is negative yaw");
  failures += check(
    !sim::wheel_targets_to_chassis(NAN, 0.2, 0.32).valid, "nan wheel safe");

  core::LineRecovery recovery({0.5, 0.07});
  recovery.update(true, -0.4, 0.02);
  const auto recovering = recovery.update(false, 0.0, 0.02);
  wheels = core::plan_wheel_speeds(
    speed, 0.25, 0.0, recovering.suggested_steering, recovering.state);
  command = sim::wheel_targets_to_chassis(wheels.left_m_s, wheels.right_m_s, 0.32);
  failures += check(recovering.state == core::LineState::RECOVERY &&
    command.linear_x > 0.0, "recovery keeps nonzero speed");
  const auto fault_wheels = core::plan_wheel_speeds(
    speed, 0.25, 0.0, 0.0, core::LineState::FAULT);
  failures += check(near(fault_wheels.left_m_s, 0.0) &&
    near(fault_wheels.right_m_s, 0.0), "fault stops");
  failures += check(
    sim::input_fault_reason(1.0, 0.7, 0.9, 0.2, 0.3) == "line sensor timeout",
    "sensor timeout");
  failures += check(
    sim::input_fault_reason(1.0, 0.9, 0.6, 0.2, 0.3) == "odometry timeout",
    "odom timeout");
  failures += check(
    !sim::input_fault_reason(1.0, 0.9, 0.9, 0.2, 0.3).size(), "fresh input");
  failures += check(
    !sim::input_fault_reason(
      NAN, 0.9, 0.9, 0.2, 0.3).empty(), "nan timing safe");
  const auto no_inputs = sim::evaluate_input_freshness(
    1.0, 0.99, 0.99, 0.2, 0.3, false);
  failures += check(
    !no_inputs.fresh && no_inputs.reason == "sensor and odometry inputs are not ready",
    "missing odom waits without qualifying for start");
  failures += check(
    !sim::evaluate_input_freshness(1.0, 0.99, 0.60, 0.2, 0.3, true).fresh,
    "stale odom cannot start");
  failures += check(
    !sim::evaluate_input_freshness(1.0, 0.70, 0.99, 0.2, 0.3, true).fresh,
    "stale sensor cannot start");
  failures += check(
    !sim::evaluate_input_freshness(0.9, 1.0, 0.8, 0.2, 0.3, true).fresh,
    "clock rewind cannot make negative age fresh");
  const auto fresh = sim::evaluate_input_freshness(
    1.0, 0.90, 0.80, 0.2, 0.3, true);
  failures += check(
    fresh.fresh && near(fresh.sensor_age_s, 0.10) && near(fresh.odom_age_s, 0.20),
    "fresh receive times qualify");
  sim::StartupFreshnessGate startup_gate(3);
  failures += check(!startup_gate.update(true, 1, 1), "one distinct pair does not start");
  failures += check(
    !startup_gate.update(true, 1, 1) && !startup_gate.update(true, 1, 1) &&
    startup_gate.consecutive_cycles() == 1,
    "one odom and sensor frame cannot be counted by three control cycles");
  failures += check(
    !startup_gate.update(true, 2, 1) && !startup_gate.update(true, 3, 1) &&
    startup_gate.consecutive_cycles() == 1,
    "three odom frames without sensor snapshots cannot start");
  failures += check(!startup_gate.update(true, 2, 2), "second distinct pair does not start");
  failures += check(startup_gate.update(true, 3, 3), "three distinct pairs start");
  failures += check(
    !startup_gate.update(true, 3, 3) && startup_gate.consecutive_cycles() == 3,
    "started gate does not consume the same pair again");
  failures += check(
    startup_gate.first_odom_sequence() == 1 &&
    startup_gate.first_sensor_sequence() == 1 &&
    startup_gate.last_odom_sequence() == 3 &&
    startup_gate.last_sensor_sequence() == 3,
    "gate records distinct startup sequence ranges");
  startup_gate.reset();
  failures += check(
    !startup_gate.update(true, 4, 4) && !startup_gate.update(false, 4, 4) &&
    startup_gate.consecutive_cycles() == 0,
    "stale input waits and resets startup count without fault");
  failures += check(
    !startup_gate.update(true, 5, 4) && startup_gate.consecutive_cycles() == 0,
    "three sensor snapshots without a new odom cannot start");

  sim::SensorSnapshotSequence snapshots;
  failures += check(
    !snapshots.update(sim::SensorSnapshotSequence::Input::VALUES, true, 1.00) &&
    !snapshots.update(sim::SensorSnapshotSequence::Input::ERROR, true, 1.01) &&
    !snapshots.update(sim::SensorSnapshotSequence::Input::DETECTED, true, 1.02) &&
    snapshots.sequence() == 0,
    "partial sensor topics do not form a snapshot");
  failures += check(
    snapshots.update(sim::SensorSnapshotSequence::Input::ACTIVATION, true, 1.03) &&
    snapshots.sequence() == 1 && near(snapshots.receive_time_s(), 1.00),
    "four fresh sensor topics form exactly one snapshot");
  failures += check(
    !snapshots.update(sim::SensorSnapshotSequence::Input::VALUES, true, 1.04) &&
    snapshots.sequence() == 1,
    "one updated sensor topic cannot form another snapshot");
  snapshots.update(sim::SensorSnapshotSequence::Input::ERROR, true, 1.05);
  snapshots.update(sim::SensorSnapshotSequence::Input::DETECTED, true, 1.06);
  failures += check(
    snapshots.update(sim::SensorSnapshotSequence::Input::ACTIVATION, true, 1.07) &&
    snapshots.sequence() == 2,
    "second complete sensor group forms one new snapshot");
  snapshots.update(sim::SensorSnapshotSequence::Input::VALUES, true, 1.08);
  snapshots.update(sim::SensorSnapshotSequence::Input::ERROR, false, 1.09);
  snapshots.update(sim::SensorSnapshotSequence::Input::DETECTED, true, 1.10);
  failures += check(
    !snapshots.update(sim::SensorSnapshotSequence::Input::ACTIVATION, true, 1.11) &&
    snapshots.sequence() == 2 && !snapshots.inputs_valid(),
    "non-finite sensor member cannot form a snapshot");
  failures += check(
    snapshots.update(sim::SensorSnapshotSequence::Input::ERROR, true, 1.12) &&
    snapshots.sequence() == 3 && snapshots.inputs_valid(),
    "new valid replacement completes the pending sensor group");
  const auto snapshot_sequence_before_start = snapshots.sequence();
  const auto snapshot_time_before_start = snapshots.receive_time_s();
  sim::StartupFreshnessGate independent_reset(1);
  independent_reset.update(true, 9, snapshot_sequence_before_start);
  independent_reset.reset();
  failures += check(
    snapshots.sequence() == snapshot_sequence_before_start &&
    near(snapshots.receive_time_s(), snapshot_time_before_start) &&
    snapshots.inputs_valid(),
    "startup controller reset leaves sensor receive state unchanged");

  const auto complete_final = [](sim::FinalLapMetrics & metrics) {
      metrics.update_stage("FINISHED");
      metrics.update_marker("A");
      metrics.update_ab_time(14.999);
      metrics.update_lap_time(89.999);
      metrics.update_ab_pass(true);
      metrics.update_lap_pass(true);
      metrics.update_final_speed(0.0);
      metrics.update_fault_reason("");
      metrics.update_recovery(0, 0.0);
    };
  sim::FinalLapMetrics final_metrics;
  final_metrics.begin();
  final_metrics.update_stage("FINISHED");
  failures += check(
    !final_metrics.evaluate().ready,
    "finished before lap metrics waits instead of failing");
  complete_final(final_metrics);
  failures += check(
    final_metrics.evaluate().ready && final_metrics.evaluate().passed,
    "consistent same-generation final metrics pass");
  failures += check(
    final_metrics.final_speed_mps() == 0.0 &&
    final_metrics.fault_reason().empty(),
    "zero recovery, zero duration, zero final speed, and empty fault are valid");

  sim::FinalLapMetrics missing_ab;
  missing_ab.begin();
  missing_ab.update_stage("FINISHED");
  missing_ab.update_marker("A");
  missing_ab.update_lap_time(20.0);
  missing_ab.update_ab_pass(true);
  missing_ab.update_lap_pass(true);
  missing_ab.update_final_speed(0.0);
  missing_ab.update_fault_reason("");
  failures += check(
    !missing_ab.evaluate().ready &&
    missing_ab.evaluate().diagnostic.find("missing_a_to_b_time_s=true") != std::string::npos,
    "missing A-B time has explicit final timeout diagnostic");

  sim::FinalLapMetrics nan_lap;
  nan_lap.begin();
  complete_final(nan_lap);
  nan_lap.update_lap_time(NAN);
  failures += check(
    nan_lap.evaluate().ready && !nan_lap.evaluate().passed &&
    nan_lap.evaluate().diagnostic.find("lap_time_finite=0") != std::string::npos,
    "NaN lap time fails with its field name");

  sim::FinalLapMetrics wrong_stage;
  wrong_stage.begin();
  complete_final(wrong_stage);
  wrong_stage.update_stage("D_TO_A");
  failures += check(!wrong_stage.evaluate().passed, "finished true with wrong stage fails");
  sim::FinalLapMetrics wrong_marker;
  wrong_marker.begin();
  complete_final(wrong_marker);
  wrong_marker.update_marker("D");
  failures += check(!wrong_marker.evaluate().passed, "last marker other than A fails");
  sim::FinalLapMetrics inconsistent_ab;
  inconsistent_ab.begin();
  complete_final(inconsistent_ab);
  inconsistent_ab.update_ab_pass(false);
  failures += check(
    !inconsistent_ab.evaluate().passed &&
    inconsistent_ab.evaluate().diagnostic.find("a_to_b_pass_consistent=0") !=
    std::string::npos,
    "A-B pass inconsistent with time fails");
  sim::FinalLapMetrics inconsistent_lap;
  inconsistent_lap.begin();
  complete_final(inconsistent_lap);
  inconsistent_lap.update_lap_pass(false);
  failures += check(
    !inconsistent_lap.evaluate().passed &&
    inconsistent_lap.evaluate().diagnostic.find("lap_pass_consistent=0") !=
    std::string::npos,
    "lap pass inconsistent with time fails");
  sim::FinalLapMetrics strict_ab;
  strict_ab.begin();
  complete_final(strict_ab);
  strict_ab.update_ab_time(15.0);
  strict_ab.update_ab_pass(false);
  failures += check(!strict_ab.evaluate().passed, "15.000 second A-B strictly fails");
  sim::FinalLapMetrics strict_lap;
  strict_lap.begin();
  complete_final(strict_lap);
  strict_lap.update_lap_time(90.0);
  strict_lap.update_lap_pass(false);
  failures += check(!strict_lap.evaluate().passed, "90.000 second lap strictly fails");
  failures += check(
    sim::input_fault_reason(1.0, 0.99, 0.69, 0.2, 0.3) == "odometry timeout",
    "running odom timeout remains enforced");
  failures += check(
    sim::input_fault_reason(1.0, 0.79, 0.99, 0.2, 0.3) == "line sensor timeout",
    "running sensor timeout remains enforced");
  failures += check(
    sim::input_fault_reason(1.02, 1.0, 1.0, 0.2, 0.3).empty(),
    "first cycle after fresh start has no timeout");

  sim::ProgressDetector detector({});
  core::ProgressStateMachine progress({0.5, 0.2, 15.0, 90.0});
  auto state = progress.update(true, core::Marker::NONE, 0.0, 0.0);
  failures += check(state.state == core::ProgressState::A_TO_B, "start A-B");
  failures += check(
    detector.update(0.0, 0.0, state.state) == core::Marker::NONE,
    "initial A cannot complete");
  failures += check(
    detector.update(0.0, 0.0, state.state) == core::Marker::NONE,
    "region triggers once");
  failures += check(
    detector.update(1.7, -1.5, state.state) == core::Marker::NONE,
    "C cannot advance A-B");
  const auto b_marker = detector.update(1.7, 0.0, state.state);
  state = progress.update(false, b_marker, 1.7, 7.0);
  failures += check(b_marker == core::Marker::B &&
    state.state == core::ProgressState::B_TO_C && state.marker_accepted,
    "B advances to B-C without stop semantics");
  const auto after_wrong = progress.update(false, core::Marker::D, 3.0, 12.0);
  failures += check(
    after_wrong.state == core::ProgressState::B_TO_C && !after_wrong.marker_accepted,
    "wrong order cannot advance");
  const auto at_c = progress.update(false, core::Marker::C, 3.4, 14.0);
  failures += check(
    at_c.state == core::ProgressState::C_TO_D && at_c.marker_accepted,
    "C advances without stop semantics");
  const auto at_d = progress.update(false, core::Marker::D, 5.0, 20.0);
  failures += check(
    at_d.state == core::ProgressState::D_TO_A && at_d.marker_accepted,
    "D advances without stop semantics");
  const auto finished = progress.update(false, core::Marker::A, 7.0, 28.0);
  failures += check(
    finished.state == core::ProgressState::FINISHED && finished.marker_accepted &&
    near(finished.ab_elapsed_s, 7.0) && near(finished.lap_elapsed_s, 28.0) &&
    finished.ab_within_target && finished.lap_within_target,
    "B-C-D-A finishes and freezes passing times");
  const auto still_finished = progress.update(false, core::Marker::B, 8.0, 40.0);
  failures += check(
    still_finished.state == core::ProgressState::FINISHED &&
    near(still_finished.ab_elapsed_s, 7.0) &&
    near(still_finished.lap_elapsed_s, 28.0),
    "FINISHED cannot advance or rewrite times");
  core::ProgressStateMachine strict_pass({0.0, 0.0, 15.0, 90.0});
  strict_pass.update(true, core::Marker::NONE, 0.0, 0.0);
  const auto ab_just_pass = strict_pass.update(false, core::Marker::B, 1.0, 14.999);
  failures += check(ab_just_pass.ab_within_target, "14.999 second A-B passes");
  strict_pass.update(false, core::Marker::C, 2.0, 20.0);
  strict_pass.update(false, core::Marker::D, 3.0, 30.0);
  const auto lap_just_pass = strict_pass.update(false, core::Marker::A, 4.0, 89.999);
  failures += check(lap_just_pass.lap_within_target, "89.999 second lap passes");
  core::ProgressStateMachine strict_fail({0.0, 0.0, 15.0, 90.0});
  strict_fail.update(true, core::Marker::NONE, 0.0, 0.0);
  const auto ab_at_limit = strict_fail.update(false, core::Marker::B, 1.0, 15.0);
  strict_fail.update(false, core::Marker::C, 2.0, 20.0);
  strict_fail.update(false, core::Marker::D, 3.0, 30.0);
  const auto lap_at_limit = strict_fail.update(false, core::Marker::A, 4.0, 90.0);
  failures += check(
    !ab_at_limit.ab_within_target && !lap_at_limit.lap_within_target,
    "exact time limits fail");
  const auto world = detector.odom_to_world(1.0, 0.2);
  failures += check(near(world.x, 1.30) && near(world.y, 2.80), "rotated odom world transform");
  failures += check(!std::isfinite(
    detector.odom_to_world(std::numeric_limits<double>::infinity(), 0.0).x),
    "invalid odom safe");
  return failures;
}
