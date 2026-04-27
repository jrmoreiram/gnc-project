#include "sensors/gps.h"

#include <random>

namespace gnc::sensors {

namespace {
std::mt19937& Rng() {
  static std::mt19937 gen(84);
  return gen;
}
}  // namespace

Gps::Gps(double pos_noise_std, double vel_noise_std, double update_rate_hz)
    : pos_noise_std_(pos_noise_std), vel_noise_std_(vel_noise_std), update_period_s_(1.0 / update_rate_hz) {}

bool Gps::update(const VehicleState& truth_state, double dt_s, GpsMeasurement& out) {
  elapsed_s_ += dt_s;
  if (elapsed_s_ < update_period_s_) {
    return false;
  }
  elapsed_s_ = 0.0;

  std::normal_distribution<double> n_pos(0.0, pos_noise_std_);
  std::normal_distribution<double> n_vel(0.0, vel_noise_std_);

  out.position_ned_m = truth_state.position_ned;
  out.velocity_ned_mps = truth_state.velocity_ned;
  for (int i = 0; i < 3; ++i) {
    out.position_ned_m(i) += n_pos(Rng());
    out.velocity_ned_mps(i) += n_vel(Rng());
  }
  out.timestamp_s = truth_state.timestamp_s;
  out.valid = true;
  return true;
}

}  // namespace gnc::sensors
