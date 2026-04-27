#ifndef GNC_SENSORS_BAROMETER_H_
#define GNC_SENSORS_BAROMETER_H_

#include "utils/types.h"

namespace gnc::sensors {

/**
 * @brief Medição de altitude barométrica.
 */
struct BarometerMeasurement {
  double altitude_m{0.0};
  double pressure_pa{0.0};
  double timestamp_s{0.0};
};

/**
 * @brief Modelo simplificado de barômetro.
 */
class Barometer {
 public:
  explicit Barometer(double altitude_noise_std);

  /**
   * @brief Mede altitude e pressão equivalente.
   */
  BarometerMeasurement sample(const VehicleState& truth_state);

 private:
  double altitude_noise_std_;
};

}  // namespace gnc::sensors

#endif  // GNC_SENSORS_BAROMETER_H_
