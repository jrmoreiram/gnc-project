#ifndef GNC_FLIGHT_LOOP_FLIGHT_LOOP_H_
#define GNC_FLIGHT_LOOP_FLIGHT_LOOP_H_

#include "control/actuator_controller.h"
#include "estimation/state_estimator.h"
#include "guidance/guidance_law.h"
#include "guidance/trajectory_planner.h"
#include "sensors/barometer.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "utils/data_logger.h"

namespace gnc::flight_loop {

/**
 * @brief Loop principal de voo com sequência Sense-Estimate-Guide-Control.
 */
class FlightLoop {
 public:
  FlightLoop();

  /**
   * @brief Executa simulação por duração definida.
   */
  void run(double duration_s, double dt_s);

 private:
  VehicleState truth_state_;
  sensors::Imu imu_;
  sensors::Gps gps_;
  sensors::Barometer baro_;
  estimation::StateEstimator estimator_;
  guidance::TrajectoryPlanner planner_;
  guidance::GuidanceLaw guidance_;
  control::ActuatorController controller_;
  utils::DataLogger logger_;

  void propagateTruth(const ActuatorCommand& cmd, double dt_s);
};

}  // namespace gnc::flight_loop

#endif  // GNC_FLIGHT_LOOP_FLIGHT_LOOP_H_
