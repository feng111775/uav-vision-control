#include "car_control/health.hpp"

int main()
{
  if (!car_control::healthy()) {
    return 1;
  }
  return car_control::version() == "0.1.0" ? 0 : 2;
}
