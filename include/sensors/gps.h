#ifndef GNC_SENSORS_GPS_H_
#define GNC_SENSORS_GPS_H_

#include "utils/types.h"

namespace gnc::sensors {

/**
 * @brief Medição de posição/velocidade fornecida pelo GPS.
 */
struct GpsMeasurement {
  Vector3 position_ned_m{Vector3::Zero()};
  Vector3 velocity_ned_mps{Vector3::Zero()};
  double timestamp_s{0.0};
  bool valid{true};
};

/**
 * @brief Simulador simplificado de GPS.
 */
class Gps {
 public:
  Gps(double pos_noise_std, double vel_noise_std, double update_rate_hz);

  /**
   * @brief Atualiza o GPS; retorna true quando há nova amostra disponível.
   */
  bool update(const VehicleState& truth_state, double dt_s, GpsMeasurement& out);

 private:
  double pos_noise_std_;
  double vel_noise_std_;
  double update_period_s_;
  double elapsed_s_{0.0};
};

}  // namespace gnc::sensors

#endif  // GNC_SENSORS_GPS_H_
