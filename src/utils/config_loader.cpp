#include "utils/config_loader.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace gnc::utils {

LoopConfig ConfigLoader::loadLoopConfig(const std::string& gnc_yaml_path) {
  const YAML::Node n = YAML::LoadFile(gnc_yaml_path);
  LoopConfig cfg;

  if (n["flight_loop"]) {
    cfg.dt_s = n["flight_loop"]["dt_s"].as<double>(cfg.dt_s);
    cfg.sim_duration_s = n["flight_loop"]["sim_duration_s"].as<double>(cfg.sim_duration_s);
  }
  return cfg;
}

std::vector<guidance::Waypoint> ConfigLoader::loadMission(const std::string& mission_yaml_path) {
  const YAML::Node n = YAML::LoadFile(mission_yaml_path);
  if (!n["waypoints"] || !n["waypoints"].IsSequence()) {
    throw std::runtime_error("mission_profile.yaml sem lista de waypoints valida");
  }

  std::vector<guidance::Waypoint> result;
  for (const auto& wp_node : n["waypoints"]) {
    guidance::Waypoint wp;
    wp.position_ned_m.x() = wp_node["north_m"].as<double>();
    wp.position_ned_m.y() = wp_node["east_m"].as<double>();
    wp.position_ned_m.z() = wp_node["down_m"].as<double>();
    wp.target_speed_mps = wp_node["target_speed_mps"].as<double>(15.0);
    result.push_back(wp);
  }
  return result;
}

}  // namespace gnc::utils
