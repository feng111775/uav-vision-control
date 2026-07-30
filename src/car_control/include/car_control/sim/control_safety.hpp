#ifndef CAR_CONTROL__SIM__CONTROL_SAFETY_HPP_
#define CAR_CONTROL__SIM__CONTROL_SAFETY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace car_control::sim
{

struct ChassisCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
  bool valid{false};
};

struct InputFreshness
{
  bool fresh{false};
  double sensor_age_s{0.0};
  double odom_age_s{0.0};
  std::string reason;
};

class StartupFreshnessGate
{
public:
  explicit StartupFreshnessGate(int required_cycles) noexcept;
  bool update(
    bool inputs_fresh, std::uint64_t odom_sequence,
    std::uint64_t sensor_sequence) noexcept;
  void reset() noexcept;
  int consecutive_cycles() const noexcept;
  std::uint64_t first_odom_sequence() const noexcept;
  std::uint64_t first_sensor_sequence() const noexcept;
  std::uint64_t last_odom_sequence() const noexcept;
  std::uint64_t last_sensor_sequence() const noexcept;
  bool valid() const noexcept;

private:
  int required_cycles_{0};
  int consecutive_cycles_{0};
  std::uint64_t first_odom_sequence_{0};
  std::uint64_t first_sensor_sequence_{0};
  std::uint64_t last_odom_sequence_{0};
  std::uint64_t last_sensor_sequence_{0};
};

class SensorSnapshotSequence
{
public:
  enum class Input : std::size_t {VALUES = 0, ERROR, DETECTED, ACTIVATION};

  bool update(Input input, bool valid, double receive_time_s) noexcept;
  bool inputs_valid() const noexcept;
  std::uint64_t sequence() const noexcept;
  double receive_time_s() const noexcept;

private:
  static constexpr std::size_t input_count_{4};
  std::array<std::uint64_t, input_count_> input_sequences_{};
  std::array<std::uint64_t, input_count_> accepted_sequences_{};
  std::array<double, input_count_> receive_times_{};
  std::array<bool, input_count_> valid_{};
  std::uint64_t snapshot_sequence_{0};
  double snapshot_receive_time_s_{0.0};
};

ChassisCommand wheel_targets_to_chassis(
  double left_m_s, double right_m_s, double wheel_separation_m) noexcept;

InputFreshness evaluate_input_freshness(
  double now_s, double sensor_stamp_s, double odom_stamp_s,
  double sensor_timeout_s, double odom_timeout_s, bool data_ready) noexcept;

std::string input_fault_reason(
  double now_s, double sensor_stamp_s, double odom_stamp_s,
  double sensor_timeout_s, double odom_timeout_s) noexcept;

}  // namespace car_control::sim

#endif
