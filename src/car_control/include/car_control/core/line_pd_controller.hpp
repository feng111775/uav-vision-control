#ifndef CAR_CONTROL__CORE__LINE_PD_CONTROLLER_HPP_
#define CAR_CONTROL__CORE__LINE_PD_CONTROLLER_HPP_

#include "car_control/core/types.hpp"

namespace car_control::core
{

struct PDConfig
{
  double kp{0.0};
  double kd{0.0};
  double output_limit{1.0};
  double derivative_filter_alpha{0.0};
};

class LinePDController
{
public:
  explicit LinePDController(PDConfig config) noexcept;
  ControlOutput update(double error, double dt_s) noexcept;
  void reset() noexcept;

private:
  bool config_valid() const noexcept;
  PDConfig config_;
  bool initialized_{false};
  double previous_error_{0.0};
  double filtered_derivative_{0.0};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__LINE_PD_CONTROLLER_HPP_
