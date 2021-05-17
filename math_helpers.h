#ifndef PATHFINDER_MATH_HELPERS_H
#define PATHFINDER_MATH_HELPERS_H

#include "vector.h"

#include <utility>

namespace pathfinder {

enum class AngleDirection {
  kNoDirection,
  kCounterclockwise,
  kClockwise
};

namespace math {

constexpr double kPi = 3.141592653589793;
constexpr double k2Pi = 6.283185307179586;
constexpr double kDoublePrecisionTolerance = 0.000001;

Vector polarToVector(const double r, const double theta);
double distance(const Vector &p1, const Vector &p2);

/**
 * Calculates a value that has the same sign of the magnitude of the cross
 * product. This can be used for determining winding order of two vectors.
 * The value returned is not the actual magnitude.
 *
 * @param v1p1 Start of the first vector
 * @param v1p2 End of the first vector
 * @param v2p1 Start of the second vector
 * @param v2p2 End of the second vector
 * @return A number with the same sign of the magnitude of the cross product of these two vectors.
 */
double crossProductForSign(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);

double dotProduct(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);
bool isPointInTriangle(const Vector &point, const Vector &triangleVertex1, const Vector &triangleVertex2, const Vector &triangleVertex3);
bool lessThan(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
bool equal(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);

/**
 * Calculate the angle of the vector point1->point2 relative to the origin (+x; East)
 *
 * @param point1 Start of the vector
 * @param point2 End of the vector
 * @return The angle relative to the origin
 */
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

/**
 * Checks if a line segment intersects with a circle.
 *
 * @param lineSegmentStartPoint Start point of line segment
 * @param lineSegmentEndPoint End point of line segment
 * @param centerOfCircle Center point of circle
 * @param circleRadius Radius of circle
 * @param[out] intersectionPoint1 If one or more intersections occurs this will be set to the first intersection point.
 *                                If only one intersection occurs, that intersection point is always set to this parameter.
 *                                In the case of 2 intersections, intersectionPoint1 is guaranteed to be closer to lineSegmentStartPoint.
 * @param[out] intersectionPoint2 If two intersections occur, this will be set to the second intersection point
 * @return The number of intersections
 */
int lineSegmentIntersectsWithCircle(Vector lineSegmentStartPoint, Vector lineSegmentEndPoint, Vector centerOfCircle, const double circleRadius, Vector *intersectionPoint1=nullptr, Vector *intersectionPoint2=nullptr);

std::pair<Vector, Vector> createVectorTangentToPointOnCircle(const Vector &circleCenter, const double circleRadius, const Vector &pointOnCircleCircumference);

/**
 * Given a `value` and a `max`, will return the `value` shifted into the
 * range [`0`, max). If `value` is greater than or equal `to` max, it will
 * reduce `value` by increments of `max` until it is less `than` max. If
 * `value` is less than 0, it will increase `value` by increments of
 * `max` until it is greater than 0.
 *
 * @param value Value to shift
 * @param max Number used to get `value` into the range [0,max)
 * @return The shifted value
 */
double normalize(double value, double max);

} // namespace math

} // namespace pathfinder

#endif // PATHFINDER_MATH_HELPERS_H
