#ifndef CAR_CONTROL__HEALTH_HPP_
#define CAR_CONTROL__HEALTH_HPP_

#include <string_view>

namespace car_control
{

std::string_view version() noexcept;
bool healthy() noexcept;

}  // namespace car_control

#endif  // CAR_CONTROL__HEALTH_HPP_
