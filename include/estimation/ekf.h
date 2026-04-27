#ifndef GNC_ESTIMATION_EKF_H_
#define GNC_ESTIMATION_EKF_H_

#include <Eigen/Dense>

#include "sensors/barometer.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "utils/types.h"

namespace gnc::estimation {

/**
 * @brief EKF de estado translacional e atitude simplificada.
 */
class Ekf {
 public:
  Ekf();

  /**
   * @brief Define o estado inicial do filtro.
   */
  void initialize(const VehicleState& initial_state);

  /**
   * @brief Etapa de predição com IMU.
   */
  void predict(const sensors::ImuMeasurement& imu, double dt_s);

  /**
   * @brief Correção com GPS.
   */
  void updateGps(const sensors::GpsMeasurement& gps);

  /**
   * @brief Correção com barômetro.
   */
  void updateBarometer(const sensors::BarometerMeasurement& baro);

  /**
   * @brief Retorna o estado estimado atual.
   */
  VehicleState getState() const;

 private:
  Eigen::Matrix<double, 9, 1> x_;
  Eigen::Matrix<double, 9, 9> P_;
  Eigen::Matrix<double, 9, 9> Q_;
  bool initialized_{false};
};

}  // namespace gnc::estimation

#endif  // GNC_ESTIMATION_EKF_H_
