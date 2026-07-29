#ifndef CAR_CONTROL__CORE__BUTTON_DEBOUNCER_HPP_
#define CAR_CONTROL__CORE__BUTTON_DEBOUNCER_HPP_

namespace car_control::core
{

class ButtonDebouncer
{
public:
  explicit ButtonDebouncer(double stable_time_s) noexcept;
  bool update(bool raw_pressed, double dt_s) noexcept;
  void reset() noexcept;

private:
  double stable_time_s_;
  bool candidate_{false};
  bool confirmed_{false};
  double candidate_time_s_{0.0};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__BUTTON_DEBOUNCER_HPP_
