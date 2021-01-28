#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <QPoint>

namespace math {

double distance(const QPointF &p1, const QPointF &p2);
double crossProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2);
double dotProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2);
bool isPointInTriangle(const QPointF &point, const QPointF &triangleVertex1, const QPointF &triangleVertex2, const QPointF &triangleVertex3);
bool lessThan(const double d1, const double d2);

} // namespace math

#endif // MATH_HELPERS_H
