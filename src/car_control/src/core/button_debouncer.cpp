#include "car_control/core/button_debouncer.hpp"

#include <cmath>

namespace car_control::core
{

ButtonDebouncer::ButtonDebouncer(double stable_time_s) noexcept
: stable_time_s_(stable_time_s) {}

bool ButtonDebouncer::update(bool raw_pressed, double dt_s) noexcept
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
    !std::isfinite(stable_time_s_) || stable_time_s_ <= 0.0)
  {
    return false;
  }
  if (raw_pressed != candidate_) {
    candidate_ = raw_pressed;
    candidate_time_s_ = 0.0;
  } else {
    candidate_time_s_ += dt_s;
  }
  if (candidate_ != confirmed_ && candidate_time_s_ >= stable_time_s_) {
    confirmed_ = candidate_;
    return confirmed_;
  }
  return false;
}

void ButtonDebouncer::reset() noexcept
{
  candidate_ = false;
  confirmed_ = false;
  candidate_time_s_ = 0.0;
}

}  // namespace car_control::core
