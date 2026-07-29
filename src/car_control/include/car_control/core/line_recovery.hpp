#ifndef CAR_CONTROL__CORE__LINE_RECOVERY_HPP_
#define CAR_CONTROL__CORE__LINE_RECOVERY_HPP_

#include "car_control/core/types.hpp"

namespace car_control::core
{

struct RecoveryConfig
{
  double fault_timeout_s{1.0};
  double search_steering{0.3};
};

struct RecoveryOutput
{
  LineState state{LineState::TRACKING};
  double suggested_steering{0.0};
  bool valid{false};
};

class LineRecovery
{
public:
  explicit LineRecovery(RecoveryConfig config) noexcept;
  RecoveryOutput update(bool line_detected, double line_error, double dt_s) noexcept;
  void reset() noexcept;

private:
  RecoveryConfig config_;
  LineState state_{LineState::TRACKING};
  double lost_time_s_{0.0};
  double last_direction_{1.0};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__LINE_RECOVERY_HPP_
