#include "car_control/core/pi_controller.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::PIConfig;
  using car_control::core::PIController;
  int failures = 0;
  PIController proportional({0.5, 0.0, -1.0, 1.0, -2.0, 2.0});
  failures += check(near(proportional.update(1.0, 0.0, 0.1).value, 0.5), "P output");
  failures += check(proportional.update(-1.0, 0.0, 0.1).value < 0.0, "reverse output");

  PIController integral({0.0, 1.0, -1.0, 1.0, -0.5, 0.5});
  failures += check(near(integral.update(1.0, 0.0, 0.1).value, 0.1), "I first");
  failures += check(near(integral.update(1.0, 0.0, 0.1).value, 0.2), "I accumulates");
  for (int index = 0; index < 20; ++index) {
    integral.update(1.0, 0.0, 0.1);
  }
  failures += check(near(integral.integral(), 0.5), "integral clamp");

  PIController saturated({2.0, 1.0, -1.0, 1.0, -5.0, 5.0});
  const auto limited = saturated.update(2.0, 0.0, 0.1);
  failures += check(limited.valid && near(limited.value, 1.0), "output saturation");
  failures += check(near(saturated.integral(), 0.0), "anti-windup");
  failures += check(!saturated.update(1.0, 0.0, 0.0).valid, "invalid dt");
  saturated.reset();
  failures += check(near(saturated.integral(), 0.0), "reset");
  PIController invalid(PIConfig{0.0, 1.0, 1.0, -1.0, -1.0, 1.0});
  failures += check(!invalid.update(1.0, 0.0, 0.1).valid, "invalid limits");
  return failures;
}
