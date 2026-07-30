#include "car_control/sim/control_safety.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::sim
{

StartupFreshnessGate::StartupFreshnessGate(int required_cycles) noexcept
: required_cycles_(required_cycles) {}

bool StartupFreshnessGate::update(
  bool inputs_fresh, std::uint64_t odom_sequence,
  std::uint64_t sensor_sequence) noexcept
{
  if (!valid() || !inputs_fresh) {
    consecutive_cycles_ = 0;
    return false;
  }
  if (odom_sequence == 0 || sensor_sequence == 0 ||
    odom_sequence <= last_odom_sequence_ ||
    sensor_sequence <= last_sensor_sequence_)
  {
    return false;
  }
  if (consecutive_cycles_ == 0) {
    first_odom_sequence_ = odom_sequence;
    first_sensor_sequence_ = sensor_sequence;
  }
  last_odom_sequence_ = odom_sequence;
  last_sensor_sequence_ = sensor_sequence;
  if (consecutive_cycles_ < required_cycles_) {
    ++consecutive_cycles_;
  }
  return consecutive_cycles_ >= required_cycles_;
}

void StartupFreshnessGate::reset() noexcept
{
  consecutive_cycles_ = 0;
  first_odom_sequence_ = 0;
  first_sensor_sequence_ = 0;
  last_odom_sequence_ = 0;
  last_sensor_sequence_ = 0;
}

int StartupFreshnessGate::consecutive_cycles() const noexcept
{
  return consecutive_cycles_;
}

std::uint64_t StartupFreshnessGate::first_odom_sequence() const noexcept
{
  return first_odom_sequence_;
}

std::uint64_t StartupFreshnessGate::first_sensor_sequence() const noexcept
{
  return first_sensor_sequence_;
}

std::uint64_t StartupFreshnessGate::last_odom_sequence() const noexcept
{
  return last_odom_sequence_;
}

std::uint64_t StartupFreshnessGate::last_sensor_sequence() const noexcept
{
  return last_sensor_sequence_;
}

bool StartupFreshnessGate::valid() const noexcept
{
  return required_cycles_ > 0;
}

bool SensorSnapshotSequence::update(
  Input input, bool valid, double receive_time_s) noexcept
{
  const auto index = static_cast<std::size_t>(input);
  if (index >= input_count_) {
    return false;
  }
  ++input_sequences_[index];
  valid_[index] = valid && std::isfinite(receive_time_s) && receive_time_s >= 0.0;
  receive_times_[index] = receive_time_s;
  if (!inputs_valid()) {
    return false;
  }
  for (std::size_t i = 0; i < input_count_; ++i) {
    if (input_sequences_[i] <= accepted_sequences_[i]) {
      return false;
    }
  }
  accepted_sequences_ = input_sequences_;
  snapshot_receive_time_s_ =
    *std::min_element(receive_times_.begin(), receive_times_.end());
  ++snapshot_sequence_;
  return true;
}

bool SensorSnapshotSequence::inputs_valid() const noexcept
{
  return std::all_of(valid_.begin(), valid_.end(), [](bool value) {return value;});
}

std::uint64_t SensorSnapshotSequence::sequence() const noexcept
{
  return snapshot_sequence_;
}

double SensorSnapshotSequence::receive_time_s() const noexcept
{
  return snapshot_receive_time_s_;
}

ChassisCommand wheel_targets_to_chassis(
  double left_m_s, double right_m_s, double wheel_separation_m) noexcept
{
  if (!std::isfinite(left_m_s) || !std::isfinite(right_m_s) ||
    !std::isfinite(wheel_separation_m) || wheel_separation_m <= 0.0)
  {
    return {};
  }
  return {
    (left_m_s + right_m_s) * 0.5,
    (right_m_s - left_m_s) / wheel_separation_m,
    true};
}

InputFreshness evaluate_input_freshness(
  double now_s, double sensor_stamp_s, double odom_stamp_s,
  double sensor_timeout_s, double odom_timeout_s, bool data_ready) noexcept
{
  InputFreshness result;
  if (!data_ready) {
    result.reason = "sensor and odometry inputs are not ready";
    return result;
  }
  if (!std::isfinite(now_s) || now_s <= 0.0 ||
    !std::isfinite(sensor_stamp_s) || !std::isfinite(odom_stamp_s) ||
    !std::isfinite(sensor_timeout_s) || !std::isfinite(odom_timeout_s) ||
    sensor_timeout_s <= 0.0 || odom_timeout_s <= 0.0 ||
    sensor_stamp_s > now_s || odom_stamp_s > now_s)
  {
    result.reason = "non-finite or invalid timing input";
    return result;
  }
  result.sensor_age_s = now_s - sensor_stamp_s;
  result.odom_age_s = now_s - odom_stamp_s;
  if (result.sensor_age_s >= sensor_timeout_s) {
    result.reason = "line sensor timeout";
    return result;
  }
  if (result.odom_age_s >= odom_timeout_s) {
    result.reason = "odometry timeout";
    return result;
  }
  result.fresh = true;
  return result;
}

std::string input_fault_reason(
  double now_s, double sensor_stamp_s, double odom_stamp_s,
  double sensor_timeout_s, double odom_timeout_s) noexcept
{
  return evaluate_input_freshness(
    now_s, sensor_stamp_s, odom_stamp_s, sensor_timeout_s, odom_timeout_s,
    true).reason;
}

}  // namespace car_control::sim
