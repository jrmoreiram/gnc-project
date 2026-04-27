#include "sensors/imu.h"

#include <random>

namespace gnc::sensors {

namespace {
std::mt19937& Rng() {
  static std::mt19937 gen(42);
  return gen;
}
}  // namespace

Imu::Imu(double accel_noise_std, double gyro_noise_std)
    : accel_noise_std_(accel_noise_std), gyro_noise_std_(gyro_noise_std) {}

ImuMeasurement Imu::sample(const VehicleState& truth_state, double /*dt_s*/) {
  std::normal_distribution<double> n_acc(0.0, accel_noise_std_);
  std::normal_distribution<double> n_gyr(0.0, gyro_noise_std_);

  ImuMeasurement m;
  // Modelo simplificado: aceleração específica estimada por variação de velocidade desprezada.
  m.accel_mps2 = Vector3(0.0, 0.0, 0.0);
  m.gyro_rps = truth_state.body_rates;

  for (int i = 0; i < 3; ++i) {
    m.accel_mps2(i) += n_acc(Rng());
    m.gyro_rps(i) += n_gyr(Rng());
  }
  m.timestamp_s = truth_state.timestamp_s;
  return m;
}

}  // namespace gnc::sensors
