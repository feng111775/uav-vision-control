#include "car_control/health.hpp"

namespace car_control
{

std::string_view version() noexcept
{
  return "0.1.0";
}

bool healthy() noexcept
{
  return true;
}

}  // namespace car_control
