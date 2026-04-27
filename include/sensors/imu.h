#ifndef GNC_SENSORS_IMU_H_
#define GNC_SENSORS_IMU_H_

#include "utils/types.h"

namespace gnc::sensors {

/**
 * @brief Leitura bruta de IMU.
 */
struct ImuMeasurement {
  Vector3 accel_mps2{Vector3::Zero()};
  Vector3 gyro_rps{Vector3::Zero()};
  double timestamp_s{0.0};
};

/**
 * @brief Simulador de IMU com ruído gaussiano.
 */
class Imu {
 public:
  Imu(double accel_noise_std, double gyro_noise_std);

  /**
   * @brief Gera uma medição sintética baseada no estado real.
   */
  ImuMeasurement sample(const VehicleState& truth_state, double dt_s);

 private:
  double accel_noise_std_;
  double gyro_noise_std_;
};

}  // namespace gnc::sensors

#endif  // GNC_SENSORS_IMU_H_
