#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <QPoint>

enum class AngleDirection {
  kNoDirection,
  kCounterclockwise,
  kClockwise
};

namespace math {

extern const double kPi;
extern const double k2Pi;

QPointF polarToVector(const double r, const double theta);
double distance(const QPointF &p1, const QPointF &p2);
double crossProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2);
double dotProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2);
bool isPointInTriangle(const QPointF &point, const QPointF &triangleVertex1, const QPointF &triangleVertex2, const QPointF &triangleVertex3);
bool lessThan(const double d1, const double d2);
bool equal(const double d1, const double d2);
double angle(const QPointF &point1, const QPointF &point2);
double angleBetweenVectors(const QPointF &v1Start, const QPointF &v1End, const QPointF &v2Start, const QPointF &v2End);
double arcAngle(const double startAngle, const double endAngle, AngleDirection direction);
double distanceBetweenEdgeAndPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &point, QPointF *pointUsedForDistanceCalculation=nullptr);
double distanceBetweenEdgeAndCircleTangentIntersectionPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &circleCenter, const double circleRadius, const AngleDirection &circleRotationDirection, QPointF *pointUsedForDistanceCalculation=nullptr);
AngleDirection angleRelativeToOrigin(double theta);
double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
std::pair<QPointF, QPointF> intersectionsPointsOfTangentLinesToCircle(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
double angle(const QPointF &point1, const AngleDirection point1Direction, const QPointF &point2, const AngleDirection point2Direction, const double circleRadius);
std::pair<QPointF, QPointF> createCircleConsciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius);


} // namespace math

#endif // MATH_HELPERS_H
