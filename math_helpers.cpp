#include "math_helpers.h"

namespace math {
  
const double kPi = 3.141592653589793;
const double k2Pi = 6.283185307179586;

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

bool equal(const double d1, const double d2) {
  return std::abs(d2-d1) < 0.000001;
}

double angle(const QPointF &point1, const QPointF &point2) {
  const double dx = point2.x()-point1.x();
  const double dy = point2.y()-point1.y();
  double angle = atan(dy/dx);
  if (dx < 0) {
    angle += kPi;
  } else if (dy < 0) {
    angle += k2Pi;
  }
  return angle;
}

double angleBetweenVectors(const QPointF &v1Start, const QPointF &v1End, const QPointF &v2Start, const QPointF &v2End) {
  const double dotProductOfVectors = dotProduct(v1Start, v1End, v2Start, v2End);
  const double lengthsMultiplied = distance(v1Start, v1End) * distance(v2Start, v2End);
  return std::acos(dotProductOfVectors/lengthsMultiplied);
}

double arcAngle(const double startAngle, const double endAngle, bool isCounterclockwise) {
  // Counterclockwise is positive
  double spanAngle;
  if (isCounterclockwise) {
    spanAngle = endAngle - startAngle;
  } else {
    spanAngle = startAngle - endAngle;
  }
  if (spanAngle < 0) {
    // Make sure it's within the range [0-2*pi)
    spanAngle += k2Pi;
  }
  if (!isCounterclockwise) {
    // Flip so that clockwise is negative
    spanAngle *= -1;
  }
  return spanAngle;
}

double distanceBetweenEdgeAndPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &point, QPointF *pointUsedForDistanceCalculation) {
  const double dx = edgeEndPoint.x()-edgeStartPoint.x();
  const double dy = edgeEndPoint.y()-edgeStartPoint.y();
  const double lengthSquared = dx*dx+dy*dy;
  if (lengthSquared == 0.0) {
    // Line segment is just a point
    return math::distance(edgeStartPoint, point);
  }
  const double t = std::clamp(static_cast<double>(((point.x()-edgeStartPoint.x())*dx + (point.y()-edgeStartPoint.y())*dy) / lengthSquared), 0.0, 1.0);
  QPointF closestPoint{edgeStartPoint.x() + t*dx, edgeStartPoint.y() + t*dy};
  if (pointUsedForDistanceCalculation != nullptr) {
    *pointUsedForDistanceCalculation = closestPoint;
  }
  return math::distance(point, closestPoint);
}

double distanceBetweenEdgeAndCircleTangentIntersectionPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &circleCenter, const double circleRadius, const AngleDirection &circleRotationDirection, QPointF *pointUsedForDistanceCalculation) {
  if (circleRotationDirection == AngleDirection::kPoint) {
    return distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleCenter, pointUsedForDistanceCalculation);
  }
  const double edgeDx = edgeEndPoint.x()-edgeStartPoint.x();
  const double edgeDy = edgeEndPoint.y()-edgeStartPoint.y();
  const double edgeLength = sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
  const double ratio = circleRadius/edgeLength;
  const QPointF circleTangentIntersectionPoint1{circleCenter.x()+edgeDx*ratio, circleCenter.y()+edgeDy*ratio};
  const QPointF circleTangentIntersectionPoint2{circleCenter.x()-edgeDx*ratio, circleCenter.y()-edgeDy*ratio};
  QPointF edgePoint1, edgePoint2;
  double dist1 = distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleTangentIntersectionPoint1, &edgePoint1);
  double dist2 = distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleTangentIntersectionPoint2, &edgePoint2);
  double distanceResult;
  if (edgePoint1 == edgePoint2) {
    // Circle is beyond the extent of the line
    // Decide which distance to use
    if (crossProduct(edgePoint1, circleTangentIntersectionPoint1, edgePoint1, circleTangentIntersectionPoint2) > 0) {
      // edgePoint1->circleTangentIntersectionPoint1 is clockwise to edgePoint1->circleTangentIntersectionPoint2
      if (circleRotationDirection == AngleDirection::kCounterclockwise) {
        distanceResult = dist2;
      } else {
        distanceResult = dist1;
      }
    } else {
      // edgePoint1->circleTangentIntersectionPoint1 is counterclockwise to edgePoint1->circleTangentIntersectionPoint2
      if (circleRotationDirection == AngleDirection::kCounterclockwise) {
        distanceResult = dist1;
      } else {
        distanceResult = dist2;
      }
    }
    // Use this point no matter what
    if (pointUsedForDistanceCalculation != nullptr) {
      *pointUsedForDistanceCalculation = edgePoint1;
    }
  } else {
    // Circle is at least partially within the extent of the line
    if (crossProduct(circleCenter, edgePoint1, circleCenter, edgePoint2) > 0) {
      // circleCenter->edgePoint1 is clockwise to circleCenter->edgePoint2
      if (circleRotationDirection == AngleDirection::kCounterclockwise) {
        distanceResult = dist1;
        if (pointUsedForDistanceCalculation != nullptr) {
          *pointUsedForDistanceCalculation = edgePoint1;
        }
      } else {
        distanceResult = dist2;
        if (pointUsedForDistanceCalculation != nullptr) {
          *pointUsedForDistanceCalculation = edgePoint2;
        }
      }
    } else {
      // circleCenter->edgePoint1 is counterclockwise to circleCenter->edgePoint2
      if (circleRotationDirection == AngleDirection::kCounterclockwise) {
        distanceResult = dist2;
        if (pointUsedForDistanceCalculation != nullptr) {
          *pointUsedForDistanceCalculation = edgePoint2;
        }
      } else {
        distanceResult = dist1;
        if (pointUsedForDistanceCalculation != nullptr) {
          *pointUsedForDistanceCalculation = edgePoint1;
        }
      }
    }
  }
  return distanceResult;
}

} // namespace math