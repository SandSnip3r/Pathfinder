#ifndef PATHFINDER_MATH_HELPERS_H
#define PATHFINDER_MATH_HELPERS_H

#include "vector.h"

#include <string_view>
#include <utility>

namespace pathfinder {

enum class AngleDirection {
  kNoDirection,
  kCounterclockwise,
  kClockwise
};

std::string_view toString(AngleDirection direction);

namespace math {

enum class IntersectionResult {
  kNone,
  kOne,
  kInfinite,
};

std::string_view toString(IntersectionResult intersectionResult);

constexpr double kPi = 3.141592653589793;
constexpr double k2Pi = 6.283185307179586;
constexpr double kDoublePrecisionTolerance = 0.000001;

Vector polarToVector(const double r, const double theta);
double distance(const Vector &p1, const Vector &p2);
double distanceSquared(const Vector &p1, const Vector &p2);

/**
 * Calculates a value that has the same sign of the magnitude of the cross
 * product. This can be used for determining winding order of two vectors.
 * The value returned is not the actual cross product; it's just the z component.
 * The magnitude might not be accurate.
 *
 * @param v1p1 Start of the first vector
 * @param v1p2 End of the first vector
 * @param v2p1 Start of the second vector
 * @param v2p2 End of the second vector
 * @return A number with the same sign of the magnitude of the cross product of these two vectors.
 */
double crossProductForSign(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);
double crossProductForSign(const Vector &v1, const Vector &v2);

double dotProduct(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2);

/**
 * Reflects point over the line segment defined by lineStart -> lineEnd.
 *
 * @param point The point to be reflected over the line
 * @param lineStart The start of the line segment
 * @param lineEnd The end of the line segment
 * @return The point that is exactly on the opposite side of the line segment.
 */
Vector reflectPointOverLine(const Vector &point, const Vector &lineStart, const Vector &lineEnd);

/**
 * Calculates if a point is on a triangle. Points exactly on an edge or
 * a vertex are considered to be on the triangle.
 *
 * @param point Point to check if it is on the triangle
 * @param triangleVertex1 Vertex 1 of the triangle
 * @param triangleVertex2 Vertex 2 of the triangle
 * @param triangleVertex3 Vertex 3 of the triangle
 * @return True if the point is on the triangle, false otherwise
 */
bool isPointOnTriangle(const Vector &point, const Vector &triangleVertex1, const Vector &triangleVertex2, const Vector &triangleVertex3);

/**
 * Checks if point is on the line segment created by lineStartEndpoint, lineEndEndpoint.
 *
 * @param point The point we're testing
 * @param lineStartEndpoint Start of the line
 * @param lineEndEndpoint End of the line
 * @return true if the point is on the line segment, false otherwise
 */
bool isPointOnLineSegment(const Vector &point, const Vector &lineStartEndpoint, const Vector &lineEndEndpoint, const double tolerance = kDoublePrecisionTolerance);

// Returns true if d1 is less than d2 with some given tolerance.
bool lessThan(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
// Returns true if d1 is greater than d2 with some given tolerance.
bool greaterThan(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
// Returns true if d1 is less than or equal to d2 with some given tolerance.
bool lessThanOrEqual(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
// Returns true if d1 is greater than or equal to d2 with some given tolerance.
bool greaterThanOrEqual(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
// Returns true if (lower <= num && num <= upper)
bool betweenOrEqual(const double num, const double lower, const double upper, const double tolerance = kDoublePrecisionTolerance);
bool equal(const double d1, const double d2, const double tolerance = kDoublePrecisionTolerance);
bool equal(const Vector &v1, const Vector &v2, const double tolerance = kDoublePrecisionTolerance);

/**
 * Calculate the angle of the vector point1->point2 relative to the origin (+x; East), increasing in the counterclockwise direction.
 *
 * @param point1 Start of the vector
 * @param point2 End of the vector
 * @return The angle relative to the origin in radians in the range [0, 2pi]
 */
double angle(const Vector &point1, const Vector &point2);

// Angle is in radians.
double angleBetweenVectors(const Vector &v1Start, const Vector &v1End, const Vector &v2Start, const Vector &v2End);
double arcAngle(const double startAngle, const double endAngle, AngleDirection direction);

double distanceBetweenEdgeAndPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr);
double distanceBetweenEdgeAndCircleTangentIntersectionPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &circleCenter, const double circleRadius, const AngleDirection &circleRotationDirection, Vector *pointUsedForDistanceCalculation=nullptr);
AngleDirection angleRelativeToOrigin(double theta);
double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const Vector &point, const Vector &centerOfCircle, const double circleRadius);

/**
 * Creates two lines which are tangent to the circle centered at `centerOfCircle` with radius `circleRadius` and intersect with `point`.
 *
 * @param point Point which the two tangents intersect with.
 * @param centerOfCircle Center of the circle for calculating tangents.
 * @param circleRadius Radius of circle.
 * @return Two points where each of the tangent lines intersect with the given circle. The first point is counterclockwise(left) of the circle from the perspective of `point`, the second point is to the clockwise(right).
 */
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

/**
 * Given a circle, defined by a center and radius, and a point on that circle's circumference, returns a line segment which is tangent to that point on the circle.
 * If the vector is perfectly vertical, it will be touching the given circle, centered on the given point, and have length 1.
 * Otherwise, the vector is on over on the y axis from x:{-1,1}. The vector extended contains the given point in the circle's circumference.
 *
 * @param circleCenter Center point of circle
 * @param circleRadius Radius of circle
 * @param pointOnCircleCircumference Point on the circumference of the given circle
 *
 * @return A pair of points defining the line segment which is tangent to the circle.
 */
std::pair<Vector, Vector> createVectorTangentToPointOnCircle(const Vector &circleCenter, const double circleRadius, Vector pointOnCircleCircumference);

/**
 * Given a line segment defined by [`lineStart`, `lineEnd`], return a perpendicular bisector with length bisectorLength.
 * The midpoint of the bisector is equal to the midpoint of the given line.
 * The bisector's direction is 90 degrees counterclockwise of the given line.
 *
 * Throws an exception if the given line has 0-length.
 *
 * @param lineStart Start of the line to bisect
 * @param lineEnd End of the line to bisect
 * @return A pair of endpoints representing the start and end, respectively, of the newly created perpendicular bisector.
 */
std::pair<Vector, Vector> createPerpendicularBisector(const Vector &lineStart, const Vector &lineEnd, double bisectorLength);

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

/**
 * Checks if two line segments intersect
 *
 * @param line1V1 First point of the first line segment
 * @param line1V2 Second point of the first line segment
 * @param line2V1 First point of the second line segment
 * @param line2V2 Second point of the second line segment
 * @param intersectionPoint An optional output holding the point of intersection when there is exactly one intersection
 * @return Either 0, 1, or infinite intersections
 */
IntersectionResult intersect(Vector line1V1, Vector line1V2, Vector line2V1, Vector line2V2, Vector *intersectionPoint=nullptr);

/**
 * Checks if two line segments intersect
 *
 * @param line1V1 First point of the first line segment
 * @param line1V2 Second point of the first line segment
 * @param line2V1 First point of the second line segment
 * @param line2V2 Second point of the second line segment
 * @param intersectionInterval1 An optional output holding a value which can be used to calculate the intersection point of the infinite line that exends beyond line segment 1.
 *                              The value is <0.0 if the intersection is beyond v1,
 *                                            0.0 if the intersection is exactly on v1,
 *                                            1.0 if the intersection is exactly on v2,
 *                                           >1.0 if the intersection is beyond v2,
 *                                         otherwise the intersection is on the line segment between the endpoints.
 * @param intersectionInterval2 An optional output holding a value which can be used to calculate the intersection point of the infinite line that exends beyond line segment 2.
 *                              The value is <0.0 if the intersection is beyond v1,
 *                                            0.0 if the intersection is exactly on v1,
 *                                            1.0 if the intersection is exactly on v2,
 *                                           >1.0 if the intersection is beyond v2,
 *                                         otherwise the intersection is on the line segment between the endpoints.
 * @return Either 0, 1, or infinite intersections
 */
IntersectionResult intersectForIntervals(Vector line1V1, Vector line1V2, Vector line2V1, Vector line2V2, double *intersectionInterval1=nullptr, double *intersectionInterval2=nullptr);

/**
 * Checks if two triangles overlap with each other
 *
 * @param triangle1V1 First vertex of the first triangle
 * @param triangle1V2 Second vertex of the first triangle
 * @param triangle1V3 Third vertex of the first triangle
 * @param triangle2V1 First vertex of the second triangle
 * @param triangle2V2 Second vertex of the second triangle
 * @param triangle2V3 Third vertex of the second triangle
 * @param onBoundary If false, only non-zero-area overlaps are considered to be overlapping
 *                   If true, zero-area overlaps are considered to be overlapping (shared edges or shared vertices)
 * @return True if the two triangles overlap, false otherwise
 */
bool trianglesOverlap(const Vector &triangle1V1, const Vector &triangle1V2, const Vector &triangle1V3, const Vector &triangle2V1, const Vector &triangle2V2, const Vector &triangle2V3, const bool onBoundary=false, const double tolerance=0.0);

/**
 * Given a line segment `point1`->`point2` and a desired line segment length `targetLength`,
 * create a new point that is `targetLength` distance away from `point1` in the same direction as `point2`
 *
 * @param point1 Start of line segment
 * @param point2 End of line segment
 * @param targetLength Desired length of new line segment
 * @return The new point
 */
Vector extendLineSegmentToLength(const Vector &point1, const Vector &point2, const double targetLength);

} // namespace math

} // namespace pathfinder

#endif // PATHFINDER_MATH_HELPERS_H
