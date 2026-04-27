#ifndef GNC_GUIDANCE_GUIDANCE_LAW_H_
#define GNC_GUIDANCE_GUIDANCE_LAW_H_

#include "guidance/trajectory_planner.h"
#include "utils/types.h"

namespace gnc::guidance {

/**
 * @brief Lei de guiamento L1 simplificada para heading e altitude.
 */
class GuidanceLaw {
 public:
  GuidanceCommand computeCommand(const VehicleState& state, const Waypoint& wp) const;
};

}  // namespace gnc::guidance

#endif  // GNC_GUIDANCE_GUIDANCE_LAW_H_
