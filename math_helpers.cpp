#include "math_helpers.h"

namespace math {
  
const double kPi = 3.141592653589793;
const double k2Pi = 6.283185307179586;

QPointF polarToVector(const double r, const double theta) {
  return {r*cos(theta), r*sin(theta)};
}

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

double arcAngle(const double startAngle, const double endAngle, AngleDirection direction) {
  if (direction == AngleDirection::kNoDirection) {
    // A point has no angle
    // TODO: Maybe even throw here since we probably shouldnt be calling this function in that case
    return 0;
  }
  // Counterclockwise is positive
  double spanAngle;
  if (direction == AngleDirection::kCounterclockwise) {
    spanAngle = endAngle - startAngle;
  } else {
    spanAngle = startAngle - endAngle;
  }
  if (spanAngle < 0) {
    // Make sure it's within the range [0-2*pi)
    spanAngle += k2Pi;
  }
  if (direction == AngleDirection::kClockwise) {
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
  if (circleRotationDirection == AngleDirection::kNoDirection) {
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

AngleDirection angleRelativeToOrigin(double theta) {
  if (theta < 0) {
    theta += 2*kPi;
  }
  if (equal(theta, 0.0) || equal(theta, kPi)) {
    return AngleDirection::kNoDirection;
  } else if (lessThan(theta, kPi)) {
    return AngleDirection::kCounterclockwise;
  } else {
    return AngleDirection::kClockwise;
  }
}


double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius) {
  // Find the two lines that are tangent to the circle and intersect with the given point
  double distanceToCircle = math::distance(point,centerOfCircle);
  double lengthOfTangentLine = sqrt(distanceToCircle*distanceToCircle-circleRadius*circleRadius);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);
  return angleOfTangentLine;
}

std::pair<QPointF, QPointF> intersectionsPointsOfTangentLinesToCircle(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius) {
  // Find the two lines that are tangent to the circle and intersect with the given point
  double distanceToCircle = math::distance(point,centerOfCircle);
  double lengthOfTangentLine = sqrt(distanceToCircle*distanceToCircle-circleRadius*circleRadius);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);

  // Calculate angle for center of circle
  double angleOfCenterOfCircle = math::angle(point, centerOfCircle);

  // First point is counterclockwise to circle
  // Second point is clockwise to circle
  double x1 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle+angleOfTangentLine);
  double y1 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle+angleOfTangentLine);

  double x2 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle-angleOfTangentLine);
  double y2 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle-angleOfTangentLine);
  return {QPointF(x1, y1), QPointF(x2, y2)};
}

double angle(const QPointF &point1, const AngleDirection point1Direction, const QPointF &point2, const AngleDirection point2Direction, const double circleRadius) {
  double angleBetweenPoints = math::angle(point1, point2);

  if (point1Direction == AngleDirection::kNoDirection && point2Direction != AngleDirection::kNoDirection) {
    // Point to circle
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(point1, point2, circleRadius);
    if (point2Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the left point
      angleBetweenPoints += angleToTangent;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the right point
      angleBetweenPoints -= angleToTangent;
    }
  } else if (point1Direction != AngleDirection::kNoDirection && point2Direction == AngleDirection::kNoDirection) {
    // Circle to point
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(point2, point1, circleRadius);
    if (point1Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the right point (from the perspective of point2)
      angleBetweenPoints -= angleToTangent;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the left point (from the perspective of point2)
      angleBetweenPoints += angleToTangent;
    }
  } else if (point1Direction != AngleDirection::kNoDirection && point2Direction != AngleDirection::kNoDirection && point1Direction != point2Direction) {
    // Circle to circle and inner tangent
    // Find the point between these two circles
    QPointF midpoint;
    midpoint.setX(point1.x() + (point2.x()-point1.x())/2);
    midpoint.setY(point1.y() + (point2.y()-point1.y())/2);
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(midpoint, point2, circleRadius);
    if (point1Direction == AngleDirection::kCounterclockwise) {
      angleBetweenPoints += angleToTangent;
    } else {
      angleBetweenPoints -= angleToTangent;
    }
  } else {
    // Point to point or circle to circle and outer tangent
  }

  return angleBetweenPoints;
}

std::pair<QPointF, QPointF> createCircleConsciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius) {
  if (circleRadius < 0.001) {
    return {point1, point2};
  }
  QPointF lineStart, lineEnd;

  if (point1Direction == AngleDirection::kNoDirection) {
    lineStart = point1;
  } else {
    // point1 is a circle
    // Does point2 lie within the first circle?
    const auto distanceBetweenPoints = math::distance(point1, point2);
    if (math::lessThan(distanceBetweenPoints, circleRadius)) {
      // TODO: point2 is inside point1's circle. Handle
      // std::cout << "point2 is inside point1's circle" << std::endl;
    } else if (math::equal(distanceBetweenPoints, circleRadius)) {
      // point2 on the circumference of point1's circle
      // the line is really not a line
      return {point2, point2};
    }
  }
  if (point2Direction == AngleDirection::kNoDirection) {
    lineEnd = point2;
  } else {
    // point2 is a circle
    // Does point1 lie within the first circle?
    const auto distanceBetweenPoints = math::distance(point1, point2);
    if (math::lessThan(distanceBetweenPoints, circleRadius)) {
      // TODO: point1 is inside point2's circle. Handle
      // std::cout << "point1 is inside point2's circle" << std::endl;
    } else if (math::equal(distanceBetweenPoints, circleRadius)) {
      // point1 on the circumference of point2's circle
      // the line is really not a line
      return {point1, point1};
    }
  }

  if (point1Direction == AngleDirection::kNoDirection && point2Direction != AngleDirection::kNoDirection) {
    // Point to circle
    const auto intersectionPoints = intersectionsPointsOfTangentLinesToCircle(point1, point2, circleRadius);
    if (point2Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the left point
      lineEnd = intersectionPoints.first;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the right point
      lineEnd = intersectionPoints.second;
    }
  } else if (point1Direction != AngleDirection::kNoDirection && point2Direction == AngleDirection::kNoDirection) {
    // Circle to point
    const auto intersectionPoints = intersectionsPointsOfTangentLinesToCircle(point2, point1, circleRadius);
    if (point1Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the right point (from the perspective of point2)
      lineStart = intersectionPoints.second;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the left point (from the perspective of point2)
      lineStart = intersectionPoints.first;
    }
  } else if (point1Direction != AngleDirection::kNoDirection && point2Direction != AngleDirection::kNoDirection) {
    // Circle to circle
    if (point1Direction == point2Direction) {
      // Outer tangents
      double distanceBetweenCircles = math::distance(point1, point2);
      double absoluteAngleToCircle2 = math::angle(point1, point2);
      double distanceToNewPoint = sqrt(distanceBetweenCircles*distanceBetweenCircles + circleRadius*circleRadius);
      double angleFromCircle2ToNewPoint = asin(circleRadius / distanceToNewPoint);
      double newAngle = absoluteAngleToCircle2 + angleFromCircle2ToNewPoint;

      double dxFromPoint1 = distanceToNewPoint * cos(newAngle);
      double dyFromPoint1 = distanceToNewPoint * sin(newAngle);

      QPointF newPoint{point1.x() + dxFromPoint1, point1.y() + dyFromPoint1};
      double circleXOffset = newPoint.x() - point2.x();
      double circleYOffset = newPoint.y() - point2.y();

      if (point1Direction == AngleDirection::kClockwise) {
        // Both clockwise
        lineStart = QPointF{point1.x() + circleXOffset, point1.y() + circleYOffset};
        lineEnd = QPointF{point2.x() + circleXOffset, point2.y() + circleYOffset};
      } else {
        // Both counterclockwise
        lineStart = QPointF{point1.x() - circleXOffset, point1.y() - circleYOffset};
        lineEnd = QPointF{point2.x() - circleXOffset, point2.y() - circleYOffset};
      }
    } else {
      // Inner tangents
      // Find the point between these two circles
      QPointF midpoint;
      midpoint.setX(point1.x() + (point2.x()-point1.x())/2);
      midpoint.setY(point1.y() + (point2.y()-point1.y())/2);
      const auto intersectionPointsWithFirstCircle = intersectionsPointsOfTangentLinesToCircle(midpoint, point1, circleRadius);
      const auto intersectionPointsWithSecondCircle = intersectionsPointsOfTangentLinesToCircle(midpoint, point2, circleRadius);
      if (point1Direction == AngleDirection::kCounterclockwise) {
        // point2 must be clockwise
        lineStart = intersectionPointsWithFirstCircle.first;
        lineEnd = intersectionPointsWithSecondCircle.first;
      } else {
        // point1 is clockwise
        // point2 must be counterclockwise
        lineStart = intersectionPointsWithFirstCircle.second;
        lineEnd = intersectionPointsWithSecondCircle.second;
      }
    }
  }

  return {lineStart, lineEnd};
}

} // namespace math