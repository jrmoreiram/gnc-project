#ifndef GNC_UTILS_TYPES_H_
#define GNC_UTILS_TYPES_H_

#include <Eigen/Dense>

namespace gnc {

/**
 * @brief Vetor 3D para coordenadas e grandezas físicas.
 */
using Vector3 = Eigen::Vector3d;

/**
 * @brief Vetor de atitude em ângulos de Euler [roll, pitch, yaw] em rad.
 */
using Attitude = Eigen::Vector3d;

/**
 * @brief Estado estimado da aeronave em NED (North-East-Down).
 */
struct VehicleState {
  Vector3 position_ned{Vector3::Zero()};
  Vector3 velocity_ned{Vector3::Zero()};
  Attitude attitude_rpy{Attitude::Zero()};
  Vector3 body_rates{Vector3::Zero()};
  double altitude_m{0.0};
  double timestamp_s{0.0};
};

/**
 * @brief Comando de guiamento para o controlador.
 */
struct GuidanceCommand {
  double target_speed_mps{0.0};
  double target_altitude_m{0.0};
  double target_heading_rad{0.0};
};

/**
 * @brief Comando final de atuadores.
 */
struct ActuatorCommand {
  double throttle{0.0};
  double aileron{0.0};
  double elevator{0.0};
  double rudder{0.0};
};

}  // namespace gnc

#endif  // GNC_UTILS_TYPES_H_
