#ifndef GNC_UTILS_DATA_LOGGER_H_
#define GNC_UTILS_DATA_LOGGER_H_

#include <fstream>
#include <string>

#include "utils/types.h"

namespace gnc::utils {

/**
 * @brief Logger CSV para análise pós-voo.
 */
class DataLogger {
 public:
  explicit DataLogger(const std::string& path = "flight_log.csv");
  ~DataLogger();

  void log(double time_s,
           const VehicleState& truth,
           const VehicleState& estimate,
           const GuidanceCommand& guidance,
           const ActuatorCommand& act);

 private:
  std::ofstream out_;
};

}  // namespace gnc::utils

#endif  // GNC_UTILS_DATA_LOGGER_H_
