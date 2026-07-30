#ifndef CAR_CONTROL__SIM__FINAL_LAP_METRICS_HPP_
#define CAR_CONTROL__SIM__FINAL_LAP_METRICS_HPP_

#include <string>

namespace car_control::sim
{

struct FinalMetricResult
{
  bool ready{false};
  bool passed{false};
  std::string diagnostic;
};

class FinalLapMetrics
{
public:
  void begin() noexcept;
  void update_stage(const std::string & value);
  void update_marker(const std::string & value);
  void update_ab_time(double value) noexcept;
  void update_lap_time(double value) noexcept;
  void update_ab_pass(bool value) noexcept;
  void update_lap_pass(bool value) noexcept;
  void update_final_speed(double value) noexcept;
  void update_fault_reason(const std::string & value);
  void update_recovery(int count, double duration_s) noexcept;
  FinalMetricResult evaluate() const;

  bool finished() const noexcept;
  const std::string & stage() const noexcept;
  const std::string & marker() const noexcept;
  double ab_time_s() const noexcept;
  double lap_time_s() const noexcept;
  bool ab_pass() const noexcept;
  bool lap_pass() const noexcept;
  double final_speed_mps() const noexcept;
  const std::string & fault_reason() const noexcept;

private:
  bool finished_{false};
  bool stage_received_{false};
  bool marker_received_{false};
  bool ab_time_received_{false};
  bool lap_time_received_{false};
  bool ab_pass_received_{false};
  bool lap_pass_received_{false};
  bool final_speed_received_{false};
  bool fault_received_{false};
  std::string stage_;
  std::string marker_;
  std::string fault_reason_;
  double ab_time_s_{0.0};
  double lap_time_s_{0.0};
  double final_speed_mps_{0.0};
  bool ab_pass_{false};
  bool lap_pass_{false};
  int recovery_count_{0};
  double recovery_duration_s_{0.0};
};

}  // namespace car_control::sim

#endif
