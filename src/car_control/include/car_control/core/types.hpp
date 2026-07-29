#ifndef CAR_CONTROL__CORE__TYPES_HPP_
#define CAR_CONTROL__CORE__TYPES_HPP_

namespace car_control::core
{

enum class LineState {TRACKING, RECOVERY, FAULT};
enum class ProgressState {WAIT_START, A_TO_B, B_TO_C, C_TO_D, D_TO_A, FINISHED};
enum class Marker {NONE, A, B, C, D};

struct ControlOutput
{
  double value{0.0};
  bool valid{false};
};

}  // namespace car_control::core

#endif  // CAR_CONTROL__CORE__TYPES_HPP_
