#ifndef CAR_CONTROL__TEST__TEST_COMMON_HPP_
#define CAR_CONTROL__TEST__TEST_COMMON_HPP_

#include <cmath>
#include <iostream>

inline int check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return 1;
  }
  return 0;
}

inline bool near(double lhs, double rhs, double tolerance = 1.0e-9)
{
  return std::abs(lhs - rhs) <= tolerance;
}

#endif  // CAR_CONTROL__TEST__TEST_COMMON_HPP_
