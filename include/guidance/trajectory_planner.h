#ifndef GNC_GUIDANCE_TRAJECTORY_PLANNER_H_
#define GNC_GUIDANCE_TRAJECTORY_PLANNER_H_

#include <cstddef>
#include <vector>

#include "utils/types.h"

namespace gnc::guidance {

/**
 * @brief Waypoint de missão.
 */
struct Waypoint {
  Vector3 position_ned_m{Vector3::Zero()};
  double target_speed_mps{15.0};
};

/**
 * @brief Planejador baseado em sequência de waypoints.
 */
class TrajectoryPlanner {
 public:
  void setWaypoints(const std::vector<Waypoint>& waypoints);
  Waypoint currentTarget(const VehicleState& state);
  bool missionCompleted() const;

 private:
  std::vector<Waypoint> waypoints_;
  std::size_t current_index_{0};
};

}  // namespace gnc::guidance

#endif  // GNC_GUIDANCE_TRAJECTORY_PLANNER_H_
