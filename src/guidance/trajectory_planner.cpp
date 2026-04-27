#include "guidance/trajectory_planner.h"

#include <cmath>

namespace gnc::guidance {

void TrajectoryPlanner::setWaypoints(const std::vector<Waypoint>& waypoints) {
  waypoints_ = waypoints;
  current_index_ = 0;
}

Waypoint TrajectoryPlanner::currentTarget(const VehicleState& state) {
  if (waypoints_.empty()) {
    return {};
  }

  const Waypoint& target = waypoints_[current_index_];
  const double dist = (target.position_ned_m - state.position_ned).norm();

  // Lógica de avanço para próximo waypoint quando dentro de raio de aceitação.
  if (dist < 15.0 && current_index_ + 1 < waypoints_.size()) {
    ++current_index_;
  }

  return waypoints_[current_index_];
}

bool TrajectoryPlanner::missionCompleted() const {
  return !waypoints_.empty() && current_index_ >= (waypoints_.size() - 1);
}

}  // namespace gnc::guidance
