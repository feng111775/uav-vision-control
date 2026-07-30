#include "car_control/sim/final_lap_metrics.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace car_control::sim
{

void FinalLapMetrics::begin() noexcept
{
  finished_ = true;
  stage_received_ = false;
  marker_received_ = false;
  ab_time_received_ = false;
  lap_time_received_ = false;
  ab_pass_received_ = false;
  lap_pass_received_ = false;
  final_speed_received_ = false;
  fault_received_ = false;
}

void FinalLapMetrics::update_stage(const std::string & value)
{
  stage_ = value;
  stage_received_ = true;
}

void FinalLapMetrics::update_marker(const std::string & value)
{
  marker_ = value;
  marker_received_ = true;
}

void FinalLapMetrics::update_ab_time(double value) noexcept
{
  ab_time_s_ = value;
  ab_time_received_ = true;
}

void FinalLapMetrics::update_lap_time(double value) noexcept
{
  lap_time_s_ = value;
  lap_time_received_ = true;
}

void FinalLapMetrics::update_ab_pass(bool value) noexcept
{
  ab_pass_ = value;
  ab_pass_received_ = true;
}

void FinalLapMetrics::update_lap_pass(bool value) noexcept
{
  lap_pass_ = value;
  lap_pass_received_ = true;
}

void FinalLapMetrics::update_final_speed(double value) noexcept
{
  final_speed_mps_ = value;
  final_speed_received_ = true;
}

void FinalLapMetrics::update_fault_reason(const std::string & value)
{
  fault_reason_ = value;
  fault_received_ = true;
}

void FinalLapMetrics::update_recovery(int count, double duration_s) noexcept
{
  recovery_count_ = count;
  recovery_duration_s_ = duration_s;
}

FinalMetricResult FinalLapMetrics::evaluate() const
{
  const std::vector<std::pair<const char *, bool>> received{
    {"stage", stage_received_}, {"last_marker", marker_received_},
    {"a_to_b_time_s", ab_time_received_}, {"lap_time_s", lap_time_received_},
    {"a_to_b_pass", ab_pass_received_}, {"lap_pass", lap_pass_received_},
    {"final_speed", final_speed_received_}, {"fault_reason", fault_received_}};
  std::ostringstream diagnostic;
  bool ready = finished_;
  diagnostic << "FINAL_METRICS:";
  for (const auto & field : received) {
    diagnostic << " missing_" << field.first << '=' << (field.second ? "false" : "true");
    ready = ready && field.second;
  }
  if (!ready) {
    return {false, false, diagnostic.str()};
  }

  const bool ab_finite = std::isfinite(ab_time_s_);
  const bool lap_finite = std::isfinite(lap_time_s_);
  const bool ab_positive = ab_finite && ab_time_s_ > 0.0;
  const bool lap_positive = lap_finite && lap_time_s_ > 0.0;
  const bool expected_ab_pass = ab_positive && ab_time_s_ < 15.0;
  const bool expected_lap_pass = lap_positive && lap_time_s_ < 90.0;
  const bool final_speed_valid =
    std::isfinite(final_speed_mps_) && std::abs(final_speed_mps_) < 1.0e-3;
  const bool recovery_valid =
    recovery_count_ >= 0 && std::isfinite(recovery_duration_s_) &&
    recovery_duration_s_ >= 0.0;
  const bool passed =
    stage_ == "FINISHED" && marker_ == "A" && ab_positive && lap_positive &&
    ab_pass_ == expected_ab_pass && lap_pass_ == expected_lap_pass &&
    expected_ab_pass && expected_lap_pass && final_speed_valid &&
    fault_reason_.empty() && recovery_valid;
  diagnostic <<
    " stage_finished=" << (stage_ == "FINISHED") <<
    " last_marker_a=" << (marker_ == "A") <<
    " a_to_b_time_finite=" << ab_finite <<
    " a_to_b_time_positive=" << ab_positive <<
    " lap_time_finite=" << lap_finite <<
    " lap_time_positive=" << lap_positive <<
    " a_to_b_pass_value=" << ab_pass_ <<
    " a_to_b_pass_consistent=" << (ab_pass_ == expected_ab_pass) <<
    " lap_pass_value=" << lap_pass_ <<
    " lap_pass_consistent=" << (lap_pass_ == expected_lap_pass) <<
    " final_speed_valid=" << final_speed_valid <<
    " fault_reason_empty=" << fault_reason_.empty() <<
    " recovery_valid=" << recovery_valid;
  return {true, passed, diagnostic.str()};
}

bool FinalLapMetrics::finished() const noexcept {return finished_;}
const std::string & FinalLapMetrics::stage() const noexcept {return stage_;}
const std::string & FinalLapMetrics::marker() const noexcept {return marker_;}
double FinalLapMetrics::ab_time_s() const noexcept {return ab_time_s_;}
double FinalLapMetrics::lap_time_s() const noexcept {return lap_time_s_;}
bool FinalLapMetrics::ab_pass() const noexcept {return ab_pass_;}
bool FinalLapMetrics::lap_pass() const noexcept {return lap_pass_;}
double FinalLapMetrics::final_speed_mps() const noexcept {return final_speed_mps_;}
const std::string & FinalLapMetrics::fault_reason() const noexcept {return fault_reason_;}

}  // namespace car_control::sim
