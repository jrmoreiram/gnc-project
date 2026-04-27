#include "utils/data_logger.h"

#include <stdexcept>

namespace gnc::utils {

DataLogger::DataLogger(const std::string& path) : out_(path) {
  if (!out_.is_open()) {
    throw std::runtime_error("Nao foi possivel abrir arquivo de log: " + path);
  }

  out_ << "t,px,py,pz,vx,vy,vz,epx,epy,epz,target_speed,target_alt,target_heading,throttle,aileron,elevator,rudder\n";
}

DataLogger::~DataLogger() {
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
}

void DataLogger::log(double time_s,
                     const VehicleState& truth,
                     const VehicleState& estimate,
                     const GuidanceCommand& guidance,
                     const ActuatorCommand& act) {
  out_ << time_s << ',' << truth.position_ned.x() << ',' << truth.position_ned.y() << ','
       << truth.position_ned.z() << ',' << truth.velocity_ned.x() << ',' << truth.velocity_ned.y() << ','
       << truth.velocity_ned.z() << ',' << estimate.position_ned.x() << ',' << estimate.position_ned.y() << ','
       << estimate.position_ned.z() << ',' << guidance.target_speed_mps << ',' << guidance.target_altitude_m
       << ',' << guidance.target_heading_rad << ',' << act.throttle << ',' << act.aileron << ','
       << act.elevator << ',' << act.rudder << '\n';
}

}  // namespace gnc::utils
