#include "flight_loop/flight_loop.h"

#include <cmath>
#include <vector>

#include "utils/config_loader.h"
#include "utils/math_utils.h"

namespace gnc::flight_loop {

FlightLoop::FlightLoop()
    : imu_(0.08, 0.01), gps_(1.8, 0.25, 5.0), baro_(0.7), logger_("flight_log.csv") {
  truth_state_.position_ned = Vector3(0.0, 0.0, -100.0);
  truth_state_.altitude_m = 100.0;
  truth_state_.velocity_ned = Vector3(15.0, 0.0, 0.0);

  estimator_.initialize(truth_state_);

  std::vector<guidance::Waypoint> mission = {
      {Vector3(300.0, 0.0, -120.0), 18.0},
      {Vector3(600.0, 250.0, -140.0), 20.0},
      {Vector3(900.0, 450.0, -130.0), 17.0},
  };
  planner_.setWaypoints(mission);
}

void FlightLoop::run(double duration_s, double dt_s) {
  double t = 0.0;

  while (t < duration_s) {
    truth_state_.timestamp_s = t;

    auto imu_meas = imu_.sample(truth_state_, dt_s);

    sensors::GpsMeasurement gps_meas;
    sensors::GpsMeasurement* gps_ptr = nullptr;
    if (gps_.update(truth_state_, dt_s, gps_meas)) {
      gps_ptr = &gps_meas;
    }

    auto baro_meas = baro_.sample(truth_state_);

    estimator_.step(imu_meas, gps_ptr, &baro_meas, dt_s);
    const VehicleState est = estimator_.state();

    const auto wp = planner_.currentTarget(est);
    const GuidanceCommand gcmd = guidance_.computeCommand(est, wp);
    const ActuatorCommand acmd = controller_.update(gcmd, est, dt_s);

    logger_.log(t, truth_state_, est, gcmd, acmd);
    propagateTruth(acmd, dt_s);

    t += dt_s;
  }
}

void FlightLoop::propagateTruth(const ActuatorCommand& cmd, double dt_s) {
  // Modelo dinâmico simplificado para manter arquitetura GNC funcional.
  truth_state_.body_rates.x() = cmd.aileron * 0.6;
  truth_state_.body_rates.y() = cmd.elevator * 0.5;
  truth_state_.body_rates.z() = cmd.rudder * 0.4;

  truth_state_.attitude_rpy += truth_state_.body_rates * dt_s;
  truth_state_.attitude_rpy.z() = utils::wrapAngle(truth_state_.attitude_rpy.z());

  const double speed = 12.0 + 10.0 * cmd.throttle;
  truth_state_.velocity_ned.x() = speed * std::cos(truth_state_.attitude_rpy.z());
  truth_state_.velocity_ned.y() = speed * std::sin(truth_state_.attitude_rpy.z());
  truth_state_.velocity_ned.z() = -truth_state_.body_rates.y() * 4.0;

  truth_state_.position_ned += truth_state_.velocity_ned * dt_s;
  truth_state_.altitude_m = -truth_state_.position_ned.z();
}

}  // namespace gnc::flight_loop
