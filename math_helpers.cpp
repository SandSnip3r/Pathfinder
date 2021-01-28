#include "math_helpers.h"

namespace math {

double distance(const QPointF &p1, const QPointF &p2) {
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  return sqrt(dx*dx+dy*dy);
}

double crossProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2) {
  const double v1_x = v1p2.x() - v1p1.x();
  const double v1_y = v1p2.y() - v1p1.y();
  const double v2_x = v2p2.x() - v2p1.x();
  const double v2_y = v2p2.y() - v2p1.y();
  return (v1_x*v2_y - v2_x*v1_y);
}

double dotProduct(const QPointF &v1p1, const QPointF &v1p2, const QPointF &v2p1, const QPointF &v2p2) {
  const double dx1 = v1p2.x()-v1p1.x();
  const double dy1 = v1p2.y()-v1p1.y();
  const double dx2 = v2p2.x()-v2p1.x();
  const double dy2 = v2p2.y()-v2p1.y();
  return dx1*dx2+dy1*dy2;
}

bool isPointInTriangle(const QPointF &point, const QPointF &triangleVertex1, const QPointF &triangleVertex2, const QPointF &triangleVertex3) {
  bool b1, b2, b3;
  b1 = (math::crossProduct(triangleVertex2, point, triangleVertex2, triangleVertex1) < 0.0f);
  b2 = (math::crossProduct(triangleVertex3, point, triangleVertex3, triangleVertex2) < 0.0f);
  b3 = (math::crossProduct(triangleVertex1, point, triangleVertex1, triangleVertex3) < 0.0f);
  return ((b1 == b2) && (b2 == b3));
}

bool lessThan(const double d1, const double d2) {
  return (d2-d1 > 0.000001);
}

} // namespace math