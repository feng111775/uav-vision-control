#include "car_control/core/progress_state_machine.hpp"

#include <cmath>

namespace car_control::core
{

ProgressStateMachine::ProgressStateMachine(ProgressConfig config) noexcept
: config_(config) {}

bool ProgressStateMachine::config_valid() const noexcept
{
  return config_.minimum_node_distance_m >= 0.0 &&
         config_.node_lockout_s >= 0.0 && config_.ab_target_s > 0.0 &&
         config_.lap_target_s > 0.0;
}

Marker ProgressStateMachine::expected_marker() const noexcept
{
  switch (state_) {
    case ProgressState::A_TO_B: return Marker::B;
    case ProgressState::B_TO_C: return Marker::C;
    case ProgressState::C_TO_D: return Marker::D;
    case ProgressState::D_TO_A: return Marker::A;
    default: return Marker::NONE;
  }
}

ProgressOutput ProgressStateMachine::output(bool accepted, bool valid) const noexcept
{
  return {
    state_, accepted, valid, ab_elapsed_s_, lap_elapsed_s_,
    ab_recorded_ && ab_elapsed_s_ < config_.ab_target_s,
    lap_recorded_ && lap_elapsed_s_ < config_.lap_target_s
  };
}

ProgressOutput ProgressStateMachine::update(
  bool start_event, Marker marker, double cumulative_distance_m,
  double competition_time_s) noexcept
{
  if (!config_valid() || !std::isfinite(cumulative_distance_m) ||
    !std::isfinite(competition_time_s) || cumulative_distance_m < 0.0 ||
    competition_time_s < 0.0)
  {
    return output(false, false);
  }
  if (state_ == ProgressState::WAIT_START) {
    if (!start_event) {
      return output(false, true);
    }
    state_ = ProgressState::A_TO_B;
    start_distance_m_ = cumulative_distance_m;
    start_time_s_ = competition_time_s;
    last_node_distance_m_ = cumulative_distance_m;
    last_node_time_s_ = competition_time_s;
    return output(false, true);
  }
  if (state_ == ProgressState::FINISHED || marker == Marker::NONE ||
    marker != expected_marker())
  {
    return output(false, true);
  }
  if (cumulative_distance_m < last_node_distance_m_ ||
    competition_time_s < last_node_time_s_)
  {
    return output(false, false);
  }
  if (cumulative_distance_m - last_node_distance_m_ <
    config_.minimum_node_distance_m ||
    competition_time_s - last_node_time_s_ < config_.node_lockout_s)
  {
    return output(false, true);
  }
  last_node_distance_m_ = cumulative_distance_m;
  last_node_time_s_ = competition_time_s;
  switch (state_) {
    case ProgressState::A_TO_B:
      ab_elapsed_s_ = competition_time_s - start_time_s_;
      ab_recorded_ = true;
      state_ = ProgressState::B_TO_C;
      break;
    case ProgressState::B_TO_C:
      state_ = ProgressState::C_TO_D;
      break;
    case ProgressState::C_TO_D:
      state_ = ProgressState::D_TO_A;
      break;
    case ProgressState::D_TO_A:
      lap_elapsed_s_ = competition_time_s - start_time_s_;
      lap_recorded_ = true;
      state_ = ProgressState::FINISHED;
      break;
    default:
      break;
  }
  return output(true, true);
}

void ProgressStateMachine::reset() noexcept
{
  state_ = ProgressState::WAIT_START;
  start_distance_m_ = 0.0;
  start_time_s_ = 0.0;
  last_node_distance_m_ = 0.0;
  last_node_time_s_ = 0.0;
  ab_elapsed_s_ = 0.0;
  lap_elapsed_s_ = 0.0;
  ab_recorded_ = false;
  lap_recorded_ = false;
}

ProgressState ProgressStateMachine::state() const noexcept
{
  return state_;
}

}  // namespace car_control::core
