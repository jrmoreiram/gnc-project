#ifndef GNC_ESTIMATION_STATE_ESTIMATOR_H_
#define GNC_ESTIMATION_STATE_ESTIMATOR_H_

#include "estimation/ekf.h"
#include "sensors/barometer.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "utils/types.h"

namespace gnc::estimation {

/**
 * @brief Camada de orquestração da estimação de estado.
 */
class StateEstimator {
 public:
  StateEstimator();

  /**
   * @brief Inicializa estimador.
   */
  void initialize(const VehicleState& initial);

  /**
   * @brief Processa todas as medições de um ciclo.
   */
  void step(const sensors::ImuMeasurement& imu,
            const sensors::GpsMeasurement* gps,
            const sensors::BarometerMeasurement* baro,
            double dt_s);

  /**
   * @brief Obtém estado estimado atual.
   */
  VehicleState state() const;

 private:
  Ekf ekf_;
};

}  // namespace gnc::estimation

#endif  // GNC_ESTIMATION_STATE_ESTIMATOR_H_
