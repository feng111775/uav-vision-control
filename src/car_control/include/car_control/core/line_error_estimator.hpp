#ifndef CAR_CONTROL__CORE__LINE_ERROR_ESTIMATOR_HPP_
#define CAR_CONTROL__CORE__LINE_ERROR_ESTIMATOR_HPP_

#include <vector>

namespace car_control::core
{

struct LineError
{
  double error{0.0};
  bool detected{false};
  double total_activation{0.0};
  bool valid{false};
};

LineError estimate_line_error(
  const std::vector<double> & normalized_values,
  const std::vector<double> & position_weights, bool black_line,
  double minimum_activation) noexcept;

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__LINE_ERROR_ESTIMATOR_HPP_
