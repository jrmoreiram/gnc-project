#include "control/actuator_controller.h"

#include <cmath>

#include "utils/math_utils.h"

namespace gnc::control {

ActuatorController::ActuatorController()
    : altitude_pid_(0.08, 0.01, 0.03, -0.5, 0.5),
      heading_pid_(1.0, 0.02, 0.08, -1.0, 1.0),
      speed_pid_(0.05, 0.02, 0.01, 0.0, 1.0) {}

ActuatorCommand ActuatorController::update(const GuidanceCommand& cmd, const VehicleState& state, double dt_s) {
  ActuatorCommand out;

  const double current_heading = state.attitude_rpy.z();
  const double heading_error = utils::wrapAngle(cmd.target_heading_rad - current_heading);

  // Gera taxas desejadas para malha interna.
  Vector3 desired_rates;
  desired_rates.x() = heading_pid_.update(0.0, -heading_error, dt_s);
  desired_rates.y() = altitude_pid_.update(cmd.target_altitude_m, state.altitude_m, dt_s);
  desired_rates.z() = 0.0;

  const Vector3 surface = rate_controller_.update(desired_rates, state.body_rates, dt_s);

  out.aileron = utils::clamp(surface.x(), -1.0, 1.0);
  out.elevator = utils::clamp(surface.y(), -1.0, 1.0);
  out.rudder = utils::clamp(surface.z(), -1.0, 1.0);

  const double speed = state.velocity_ned.head<2>().norm();
  out.throttle = speed_pid_.update(cmd.target_speed_mps, speed, dt_s);

  return out;
}

}  // namespace gnc::control
