#include "car_control/core/line_pd_controller.hpp"
#include "test_common.hpp"

int main()
{
  using car_control::core::LinePDController;
  int failures = 0;
  LinePDController controller({1.0, 0.1, 1.0, 0.0});
  const auto first = controller.update(0.2, 0.1);
  failures += check(first.valid && near(first.value, 0.2), "first sample has no derivative kick");
  failures += check(near(controller.update(0.4, 0.1).value, 0.6), "derivative term");
  failures += check(near(controller.update(10.0, 0.1).value, 1.0), "output limit");
  failures += check(!controller.update(0.0, 0.0).valid, "invalid dt");
  controller.reset();
  failures += check(near(controller.update(-0.3, 0.1).value, -0.3), "reset and reverse");
  LinePDController filtered({0.0, 1.0, 10.0, 0.5});
  filtered.update(0.0, 0.1);
  failures += check(near(filtered.update(1.0, 0.1).value, 5.0), "derivative filter");
  return failures;
}
