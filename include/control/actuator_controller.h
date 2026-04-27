#ifndef GNC_CONTROL_ACTUATOR_CONTROLLER_H_
#define GNC_CONTROL_ACTUATOR_CONTROLLER_H_

#include "control/pid_controller.h"
#include "control/rate_controller.h"
#include "utils/types.h"

namespace gnc::control {

/**
 * @brief Controlador de atitude/altitude para geração de comandos de atuadores.
 */
class ActuatorController {
 public:
  ActuatorController();

  /**
   * @brief Converte comando de guiamento em comando de atuador.
   */
  ActuatorCommand update(const GuidanceCommand& cmd, const VehicleState& state, double dt_s);

 private:
  PidController altitude_pid_;
  PidController heading_pid_;
  PidController speed_pid_;
  RateController rate_controller_;
};

}  // namespace gnc::control

#endif  // GNC_CONTROL_ACTUATOR_CONTROLLER_H_
