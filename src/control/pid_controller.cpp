#include "control/pid_controller.h"

#include "utils/math_utils.h"

namespace gnc::control {

PidController::PidController(double kp, double ki, double kd, double min_out, double max_out)
    : kp_(kp), ki_(ki), kd_(kd), min_out_(min_out), max_out_(max_out) {}

double PidController::update(double setpoint, double measurement, double dt_s) {
  const double error = setpoint - measurement;

  if (first_) {
    prev_error_ = error;
    first_ = false;
  }

  integral_ += error * dt_s;
  integral_ = utils::clamp(integral_, min_out_, max_out_);

  const double derivative = (error - prev_error_) / (dt_s > 1e-6 ? dt_s : 1e-6);
  prev_error_ = error;

  const double u = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return utils::clamp(u, min_out_, max_out_);
}

void PidController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_ = true;
}

}  // namespace gnc::control
