#include "guidance/guidance_law.h"

#include <cmath>

#include "utils/math_utils.h"

namespace gnc::guidance {

GuidanceCommand GuidanceLaw::computeCommand(const VehicleState& state, const Waypoint& wp) const {
  GuidanceCommand cmd;

  Vector3 delta = wp.position_ned_m - state.position_ned;
  const double desired_heading = std::atan2(delta.y(), delta.x());

  cmd.target_heading_rad = utils::wrapAngle(desired_heading);
  cmd.target_altitude_m = -wp.position_ned_m.z();
  cmd.target_speed_mps = wp.target_speed_mps;

  return cmd;
}

}  // namespace gnc::guidance
