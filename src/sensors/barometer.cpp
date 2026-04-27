#include "sensors/barometer.h"

#include <cmath>
#include <random>

#include "constants.h"

namespace gnc::sensors {

namespace {
std::mt19937& Rng() {
  static std::mt19937 gen(126);
  return gen;
}
}  // namespace

Barometer::Barometer(double altitude_noise_std) : altitude_noise_std_(altitude_noise_std) {}

BarometerMeasurement Barometer::sample(const VehicleState& truth_state) {
  std::normal_distribution<double> noise(0.0, altitude_noise_std_);

  BarometerMeasurement m;
  m.altitude_m = truth_state.altitude_m + noise(Rng());
  m.pressure_pa = constants::kSeaLevelPressurePa *
                  std::pow(1.0 - (0.0065 * m.altitude_m) / constants::kSeaLevelTempK, 5.255);
  m.timestamp_s = truth_state.timestamp_s;
  return m;
}

}  // namespace gnc::sensors
