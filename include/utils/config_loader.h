#ifndef GNC_UTILS_CONFIG_LOADER_H_
#define GNC_UTILS_CONFIG_LOADER_H_

#include <string>
#include <vector>

#include "guidance/trajectory_planner.h"

namespace gnc::utils {

/**
 * @brief Configuração principal de malha de voo.
 */
struct LoopConfig {
  double dt_s{0.01};
  double sim_duration_s{60.0};
};

/**
 * @brief Loader de arquivos YAML de configuração.
 */
class ConfigLoader {
 public:
  static LoopConfig loadLoopConfig(const std::string& gnc_yaml_path);
  static std::vector<guidance::Waypoint> loadMission(const std::string& mission_yaml_path);
};

}  // namespace gnc::utils

#endif  // GNC_UTILS_CONFIG_LOADER_H_
