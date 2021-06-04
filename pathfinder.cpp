#include "debuglogger.h"
#include "pathfinder.h"
#include "vector.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <string>

namespace pathfinder {

PathSegment::~PathSegment() {} // TODO: Why is this here? Why not funnel.cpp?

/* ========================================================================== **
** ============================= Free functions ============================= **
** ========================================================================== */

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path) {
  // std::cout << "    calculatePathLength: "; //DEBUGPRINTS
  double totalDistance=0;
  for (std::size_t i=0; i<path.size(); ++i) {
    const PathSegment *segment = path.at(i).get();
    const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(segment);
    const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(segment);
    if (straightSegment != nullptr) {
      totalDistance += math::distance(straightSegment->startPoint, straightSegment->endPoint);
      // std::cout << "+straight [" << totalDistance << "], "; //DEBUGPRINTS
    } else if (arcSegment != nullptr) {
      if (i != path.size()-1) {
        // If the last segment is an arc, i think its probably an unfinished path and we shouldnt add this segment
        double angle = math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, arcSegment->angleDirection);
        totalDistance += arcSegment->circleRadius * std::abs(angle);
        // std::cout << "+arc (angle start:" << arcSegment->startAngle << ", angle end: " << arcSegment->endAngle << ", angle: " << angle << ", radius:" << arcSegment->circleRadius << ") [" << totalDistance << "], "; //DEBUGPRINTS
      }
    }
  }
  // std::cout << std::endl; //DEBUGPRINTS
  return totalDistance;
}

} // namespace pathfinder
