#include "utils/math_utils.h"

#include <cmath>

namespace gnc::utils {

double clamp(double value, double min_v, double max_v) {
  if (value < min_v) return min_v;
  if (value > max_v) return max_v;
  return value;
}

double wrapAngle(double angle_rad) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

}  // namespace gnc::utils
