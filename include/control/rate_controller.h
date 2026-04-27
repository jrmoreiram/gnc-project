#ifndef GNC_CONTROL_RATE_CONTROLLER_H_
#define GNC_CONTROL_RATE_CONTROLLER_H_

#include "control/pid_controller.h"
#include "utils/types.h"

namespace gnc::control {

/**
 * @brief Controlador interno de taxas corporais (roll/pitch/yaw rate).
 */
class RateController {
 public:
  RateController();

  /**
   * @brief Gera comandos de superfícies para seguir taxas desejadas.
   */
  Vector3 update(const Vector3& desired_rates, const Vector3& measured_rates, double dt_s);

 private:
  PidController roll_rate_pid_;
  PidController pitch_rate_pid_;
  PidController yaw_rate_pid_;
};

}  // namespace gnc::control

#endif  // GNC_CONTROL_RATE_CONTROLLER_H_
