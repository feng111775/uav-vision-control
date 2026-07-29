#include "car_control/core/line_error_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::core
{

LineError estimate_line_error(
  const std::vector<double> & normalized_values,
  const std::vector<double> & position_weights, bool black_line,
  double minimum_activation) noexcept
{
  if (normalized_values.empty() ||
    normalized_values.size() != position_weights.size() ||
    !std::isfinite(minimum_activation) || minimum_activation < 0.0)
  {
    return {};
  }
  double activation_sum = 0.0;
  double weighted_sum = 0.0;
  double maximum_position = 0.0;
  for (std::size_t index = 0; index < normalized_values.size(); ++index) {
    if (!std::isfinite(normalized_values[index]) ||
      !std::isfinite(position_weights[index]))
    {
      return {};
    }
    const double value = std::clamp(normalized_values[index], 0.0, 1.0);
    const double activation = black_line ? 1.0 - value : value;
    activation_sum += activation;
    weighted_sum += activation * position_weights[index];
    maximum_position = std::max(maximum_position, std::abs(position_weights[index]));
  }
  if (activation_sum < minimum_activation || activation_sum <= 0.0 ||
    maximum_position <= 0.0)
  {
    return {0.0, false, activation_sum, true};
  }
  const double error = std::clamp(
    weighted_sum / activation_sum / maximum_position, -1.0, 1.0);
  return {error, true, activation_sum, true};
}

}  // namespace car_control::core
