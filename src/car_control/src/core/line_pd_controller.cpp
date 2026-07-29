#include "car_control/core/line_pd_controller.hpp"

#include <algorithm>
#include <cmath>

namespace car_control::core
{

LinePDController::LinePDController(PDConfig config) noexcept
: config_(config) {}

bool LinePDController::config_valid() const noexcept
{
  return std::isfinite(config_.kp) && std::isfinite(config_.kd) &&
         std::isfinite(config_.output_limit) && config_.output_limit > 0.0 &&
         config_.derivative_filter_alpha >= 0.0 &&
         config_.derivative_filter_alpha < 1.0;
}

ControlOutput LinePDController::update(double error, double dt_s) noexcept
{
  if (!config_valid() || !std::isfinite(error) ||
    !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    return {};
  }
  double derivative = 0.0;
  if (initialized_) {
    const double raw = (error - previous_error_) / dt_s;
    filtered_derivative_ =
      config_.derivative_filter_alpha * filtered_derivative_ +
      (1.0 - config_.derivative_filter_alpha) * raw;
    derivative = filtered_derivative_;
  } else {
    initialized_ = true;
  }
  previous_error_ = error;
  const double output = std::clamp(
    config_.kp * error + config_.kd * derivative,
    -config_.output_limit, config_.output_limit);
  return {output, true};
}

void LinePDController::reset() noexcept
{
  initialized_ = false;
  previous_error_ = 0.0;
  filtered_derivative_ = 0.0;
}

}  // namespace car_control::core
