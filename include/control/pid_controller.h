#ifndef GNC_CONTROL_PID_CONTROLLER_H_
#define GNC_CONTROL_PID_CONTROLLER_H_

namespace gnc::control {

/**
 * @brief Controlador PID com anti-windup por saturação de integrador.
 */
class PidController {
 public:
  PidController(double kp, double ki, double kd, double min_out, double max_out);

  /**
   * @brief Atualiza o PID e retorna saída limitada.
   */
  double update(double setpoint, double measurement, double dt_s);

  void reset();

 private:
  double kp_;
  double ki_;
  double kd_;
  double min_out_;
  double max_out_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool first_{true};
};

}  // namespace gnc::control

#endif  // GNC_CONTROL_PID_CONTROLLER_H_
