#include "car_control/core/button_debouncer.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::ButtonDebouncer;
  int failures = 0;
  ButtonDebouncer button(0.05);
  failures += check(!button.update(true, 0.01), "bounce begins");
  failures += check(!button.update(false, 0.01), "bounce cancels");
  failures += check(!button.update(true, 0.01), "stable press begins");
  failures += check(!button.update(true, 0.02), "not stable yet");
  failures += check(button.update(true, 0.03), "one press event");
  failures += check(!button.update(true, 0.20), "long press no repeat");
  button.update(false, 0.01);
  button.update(false, 0.05);
  button.update(true, 0.01);
  failures += check(button.update(true, 0.05), "second press after release");
  button.reset();
  failures += check(!button.update(true, 0.01), "reset waits for debounce");
  failures += check(!button.update(true, 0.0), "invalid dt no event");
  return failures;
}
