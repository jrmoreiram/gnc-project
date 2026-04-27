#include "control/rate_controller.h"

namespace gnc::control {

RateController::RateController()
    : roll_rate_pid_(0.10, 0.02, 0.005, -1.0, 1.0),
      pitch_rate_pid_(0.12, 0.02, 0.006, -1.0, 1.0),
      yaw_rate_pid_(0.08, 0.01, 0.003, -1.0, 1.0) {}

Vector3 RateController::update(const Vector3& desired_rates, const Vector3& measured_rates, double dt_s) {
  Vector3 cmd;
  cmd.x() = roll_rate_pid_.update(desired_rates.x(), measured_rates.x(), dt_s);
  cmd.y() = pitch_rate_pid_.update(desired_rates.y(), measured_rates.y(), dt_s);
  cmd.z() = yaw_rate_pid_.update(desired_rates.z(), measured_rates.z(), dt_s);
  return cmd;
}

}  // namespace gnc::control
