#ifndef CAR_CONTROL__CORE__PI_CONTROLLER_HPP_
#define CAR_CONTROL__CORE__PI_CONTROLLER_HPP_

#include "car_control/core/types.hpp"

namespace car_control::core
{

struct PIConfig
{
  double kp{0.0};
  double ki{0.0};
  double output_min{-1.0};
  double output_max{1.0};
  double integral_min{-1.0};
  double integral_max{1.0};
};

class PIController
{
public:
  explicit PIController(PIConfig config) noexcept;
  ControlOutput update(double target_m_s, double actual_m_s, double dt_s) noexcept;
  void reset() noexcept;
  double integral() const noexcept;

private:
  bool config_valid() const noexcept;
  PIConfig config_;
  double integral_{0.0};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__PI_CONTROLLER_HPP_
