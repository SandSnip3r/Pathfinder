#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include "vector.h"

#include <utility>

namespace pathfinder {

enum class AngleDirection {
  kNoDirection,
  kCounterclockwise,
  kClockwise
};

namespace math {

extern const double kPi;
extern const double k2Pi;
extern const double kDoublePrecisionTolerance;

Vector polarToVector(const double r, const double theta);
double distance(const Vector &p1, const Vector &p2);
double crossProduct(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);
double dotProduct(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);
bool isPointInTriangle(const Vector &point, const Vector &triangleVertex1, const Vector &triangleVertex2, const Vector &triangleVertex3);
bool lessThan(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
bool equal(const double d1, const double d2);
double angle(const Vector &point1, const Vector &point2);
double angleBetweenVectors(const Vector &v1Start, const Vector &v1End, const Vector &v2Start, const Vector &v2End);
double arcAngle(const double startAngle, const double endAngle, AngleDirection direction);
double distanceBetweenEdgeAndPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr);
double distanceBetweenEdgeAndCircleTangentIntersectionPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &circleCenter, const double circleRadius, const AngleDirection &circleRotationDirection, Vector *pointUsedForDistanceCalculation=nullptr);
AngleDirection angleRelativeToOrigin(double theta);
double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const Vector &point, const Vector &centerOfCircle, const double circleRadius);
std::pair<Vector, Vector> intersectionsPointsOfTangentLinesToCircle(const Vector &point, const Vector &centerOfCircle, const double circleRadius);
double angle(const Vector &point1, const AngleDirection point1Direction, const Vector &point2, const AngleDirection point2Direction, const double circleRadius);
std::pair<Vector, Vector> createCircleConsciousLine(const Vector &point1, const AngleDirection &point1Direction, const Vector &point2, const AngleDirection &point2Direction, const double circleRadius);


} // namespace math

} // namespace pathfinder

#endif // MATH_HELPERS_H
