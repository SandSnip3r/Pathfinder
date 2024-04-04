#include "debuglogger.h"
#include "pathfinder.h"
#include "vector.h"

#include <cstdlib>
#include <memory>
#include <vector>

namespace pathfinder {

PathSegment::~PathSegment() {} // TODO: Why is this here? Why not funnel.cpp?

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path) {
  double totalDistance=0;
  for (std::size_t i=0; i<path.size(); ++i) {
    const PathSegment *segment = path.at(i).get();
    const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(segment);
    const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(segment);
    if (straightSegment != nullptr) {
      totalDistance += math::distance(straightSegment->startPoint, straightSegment->endPoint);
    } else if (arcSegment != nullptr) {
      if (i != path.size()-1) {
        // If the last segment is an arc, i think its probably an unfinished path and we shouldnt add this segment
        const double angle = math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, arcSegment->angleDirection);
        totalDistance += arcSegment->circleRadius * std::abs(angle);
      }
    }
  }
  return totalDistance;
}

namespace internal {

std::string toString(RelativePositionToInterval pos) {
  if (pos == RelativePositionToInterval::kIsLeftOf) {
    return "kIsLeftOf";
  }
  if (pos == RelativePositionToInterval::kIsRightOf) {
    return "kIsRightOf";
  }
  // if (pos == RelativePositionToInterval::kIsWithin)
  return "kIsWithin";
}

bool lineActuallyIntersectedWithCircle(const pathfinder::Vector &lineSegmentStartPoint,
                                       const pathfinder::Vector &lineSegmentEndPoint,
                                       const pathfinder::Vector &centerOfCircle,
                                       const double circleRadius,
                                       int intersectionCount,
                                       const pathfinder::Vector &intersectionPoint1,
                                       const pathfinder::Vector &intersectionPoint2) {
  if (VLOG_IS_ON(1)) {
    std::stringstream ss;
    ss << "Checking for actual intersection. Intersection count " << intersectionCount;
    if (intersectionCount > 0) {
      ss << ", intersection 1: " << intersectionPoint1.x() << ',' << intersectionPoint1.y();
      if (intersectionCount > 1) {
        ss << ", intersection 2: " << intersectionPoint2.x() << ',' << intersectionPoint2.y();
      }
    }
    ss << ", circle center: " << centerOfCircle.x() << ',' << centerOfCircle.y() << " with radius " << circleRadius;
    VLOG(1) << ss.str();
  }
  // This function filters out intersections which are tangential and can be ignored.
  // If there's one intersection point and the two endpoints are outside, then its a single tangential intersection.
  // If there are are two intersection points and they're basically the same point, then its a single tangential intersection.
  if (intersectionCount == 0) {
    return false;
  }
  VLOG(1) << "intersectionCount == " << intersectionCount;
  if (intersectionCount == 1) {
    const auto distanceFromCircleToLineSegmentStartPoint = pathfinder::math::distance(centerOfCircle, lineSegmentStartPoint);
    const auto distanceFromCircleToLineSegmentEndPoint = pathfinder::math::distance(centerOfCircle, lineSegmentEndPoint);
    if (pathfinder::math::lessThan(circleRadius, distanceFromCircleToLineSegmentStartPoint) &&
        pathfinder::math::lessThan(circleRadius, distanceFromCircleToLineSegmentEndPoint)) {
      // Both points of the tested line segment are outside the circle, this is a tangential intersection
      VLOG(1) << "Both points of the tested line segment are outside the circle, this is a tangential intersection";
      return false;
    } else if (pathfinder::math::equal(distanceFromCircleToLineSegmentStartPoint, circleRadius)) {
      // When an endpoint of the line is exactly on the circle, we want to find out if this intersection is a tangential one. To do so, we'll check the angle between the given line and the line along the radius of the circle to the point which lies on the circle.
      const auto angleOfLineRelativeToRadiusAtIntersection = pathfinder::math::angleBetweenVectors(lineSegmentStartPoint, lineSegmentEndPoint, lineSegmentStartPoint, centerOfCircle);
      if (pathfinder::math::equal(angleOfLineRelativeToRadiusAtIntersection, pathfinder::math::kPi/2.0) || pathfinder::math::equal(angleOfLineRelativeToRadiusAtIntersection, pathfinder::math::kPi*3/2.0)) {
        // This intersection is tangential!
        return false;
      }
    } else if (pathfinder::math::equal(distanceFromCircleToLineSegmentEndPoint, circleRadius)) {
      // When an endpoint of the line is exactly on the circle, we want to find out if this intersection is a tangential one. To do so, we'll check the angle between the given line and the line along the radius of the circle to the point which lies on the circle.
      const auto angleOfLineRelativeToRadiusAtIntersection = pathfinder::math::angleBetweenVectors(lineSegmentEndPoint, lineSegmentStartPoint, lineSegmentEndPoint, centerOfCircle);
      if (pathfinder::math::equal(angleOfLineRelativeToRadiusAtIntersection, pathfinder::math::kPi/2.0) || pathfinder::math::equal(angleOfLineRelativeToRadiusAtIntersection, pathfinder::math::kPi*3/2.0)) {
        // This intersection is tangential!
        return false;
      }
    } else if (pathfinder::math::lessThan(distanceFromCircleToLineSegmentStartPoint, circleRadius) &&
               pathfinder::math::lessThan(distanceFromCircleToLineSegmentEndPoint, circleRadius)) {
      // Both points of the tested line segment are inside the circle, no intersection.
      VLOG(1) << "Both points of the tested line segment are inside the circle, no intersection.";
      return false;
    } else {
      // One point is inside the circle, the other is outside.
      VLOG(1) << "One point is inside the circle, the other is outside. " << distanceFromCircleToLineSegmentStartPoint << ", " << distanceFromCircleToLineSegmentEndPoint;
    }
  } else if (intersectionCount == 2) {
    const double kPointDistanceTolerance{circleRadius * 1.3e-06};
    if (pathfinder::math::equal(intersectionPoint1, intersectionPoint2, kPointDistanceTolerance)) {
      // Points are the same, it must be a tangential intersection
      VLOG(1) << "Points are the same, it must be a tangential intersection";
      return false;
    }
  }
  return true;
}

RAIIPrinter::RAIIPrinter(const std::string &msg) : msg_(msg) {}

RAIIPrinter::~RAIIPrinter() {
  VLOG(1) << msg_;
  // if (VLOG_IS_ON(1)) {
  //   std::cout << msg_ << std::endl;
  // }
}

} // namespace internal

} // namespace pathfinder
