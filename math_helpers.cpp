#include "math_helpers.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace pathfinder {

namespace math {

Vector polarToVector(const double r, const double theta) {
  return {r*cos(theta), r*sin(theta)};
}

double distance(const Vector &p1, const Vector &p2) {
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  return std::sqrt(dx*dx+dy*dy);
}

double distanceSquared(const Vector &p1, const Vector &p2) {
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  return dx*dx+dy*dy;
}

double crossProductForSign(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2) {
  const double v1_x = v1p2.x() - v1p1.x();
  const double v1_y = v1p2.y() - v1p1.y();
  const double v2_x = v2p2.x() - v2p1.x();
  const double v2_y = v2p2.y() - v2p1.y();
  return (v1_x*v2_y - v2_x*v1_y);
}

double dotProduct(const Vector &v1p1, const Vector &v1p2, const Vector &v2p1, const Vector &v2p2) {
  const double dx1 = v1p2.x()-v1p1.x();
  const double dy1 = v1p2.y()-v1p1.y();
  const double dx2 = v2p2.x()-v2p1.x();
  const double dy2 = v2p2.y()-v2p1.y();
  return dx1*dx2+dy1*dy2;
}

bool isPointOnTriangle(const Vector &point, const Vector &triangleVertex1, const Vector &triangleVertex2, const Vector &triangleVertex3) {
  bool b1, b2, b3;
  b1 = (math::crossProductForSign(triangleVertex2, point, triangleVertex2, triangleVertex1) < 0.0f);
  b2 = (math::crossProductForSign(triangleVertex3, point, triangleVertex3, triangleVertex2) < 0.0f);
  b3 = (math::crossProductForSign(triangleVertex1, point, triangleVertex1, triangleVertex3) < 0.0f);
  return ((b1 == b2) && (b2 == b3));
}

bool isPointOnLineSegment(const Vector &point, const Vector &lineStartEndpoint, const Vector &lineEndEndpoint, const double tolerance) {
#if true

  // Sum of distance to the two endpoints is equal to the length of the line segment
  return (equal(distance(lineStartEndpoint, lineEndEndpoint),
                distance(lineStartEndpoint, point) + distance(point, lineEndEndpoint),
                tolerance));

#else

  const double dxToPoint = point.x() - lineStartEndpoint.x();
  const double dyToPoint = point.y() - lineStartEndpoint.y();
  const auto distanceToLineEndEndpoint = distanceSquared(lineStartEndpoint, lineEndEndpoint);
  if (equal(distanceToLineEndEndpoint, 0.0, 1e-10)) {
    return true;
  }
  const auto distanceToPoint = distanceSquared(lineStartEndpoint, point);
  if (equal(distanceToPoint, 0.0, 1e-10)) {
    return true;
  }
  const double ratio = std::sqrt(distanceToLineEndEndpoint / distanceToPoint);
  if (lessThan(ratio, 1.0)) {
    return false;
  }
  return equal(lineEndEndpoint, {lineStartEndpoint.x() + ratio * dxToPoint, lineStartEndpoint.y() + ratio * dyToPoint}, 1e-10);

#endif
}

bool lessThan(const double d1, const double d2, const double tolerance) {
  return (d2-d1 >= tolerance);
}

bool equal(const double d1, const double d2, const double tolerance) {
  return std::abs(d2-d1) < tolerance;
}

bool equal(const Vector &v1, const Vector &v2, const double tolerance) {
  return equal(v1.x(), v2.x(), tolerance) && equal(v1.y(), v2.y(), tolerance);
}

double angle(const Vector &point1, const Vector &point2) {
  const double dx = point2.x()-point1.x();
  const double dy = point2.y()-point1.y();
  double angle = std::atan(dy/dx);
  if (lessThan(dx, 0)) {
    angle += kPi;
  } else if (lessThan(dy, 0)) {
    angle += k2Pi;
  }
  return angle;
}

double angleBetweenVectors(const Vector &v1Start, const Vector &v1End, const Vector &v2Start, const Vector &v2End) {
  const double dotProductOfVectors = dotProduct(v1Start, v1End, v2Start, v2End);
  const double lengthsMultiplied = distance(v1Start, v1End) * distance(v2Start, v2End);
  return std::acos(dotProductOfVectors/lengthsMultiplied);
}

double arcAngle(const double startAngle, const double endAngle, AngleDirection direction) {
  if (direction == AngleDirection::kNoDirection) {
    // A point has no angle
    throw std::invalid_argument("math::arcAngle no direction given");
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

double distanceBetweenEdgeAndPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &point, Vector *pointUsedForDistanceCalculation) {
  const double dx = edgeEndPoint.x()-edgeStartPoint.x();
  const double dy = edgeEndPoint.y()-edgeStartPoint.y();
  const double lengthSquared = dx*dx+dy*dy;
  if (lengthSquared == 0.0) {
    // Line segment is just a point
    return math::distance(edgeStartPoint, point);
  }
  const double t = std::clamp(static_cast<double>(((point.x()-edgeStartPoint.x())*dx + (point.y()-edgeStartPoint.y())*dy) / lengthSquared), 0.0, 1.0);
  Vector closestPoint{edgeStartPoint.x() + t*dx, edgeStartPoint.y() + t*dy};
  if (pointUsedForDistanceCalculation != nullptr) {
    *pointUsedForDistanceCalculation = closestPoint;
  }
  return math::distance(point, closestPoint);
}

double distanceBetweenEdgeAndCircleTangentIntersectionPoint(const Vector &edgeStartPoint, const Vector &edgeEndPoint, const Vector &circleCenter, const double circleRadius, const AngleDirection &circleRotationDirection, Vector *pointUsedForDistanceCalculation) {
  if (circleRotationDirection == AngleDirection::kNoDirection) {
    return distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleCenter, pointUsedForDistanceCalculation);
  }
  const double edgeDx = edgeEndPoint.x()-edgeStartPoint.x();
  const double edgeDy = edgeEndPoint.y()-edgeStartPoint.y();
  const double edgeLength = std::sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
  const double ratio = circleRadius/edgeLength;
  const Vector circleTangentIntersectionPoint1{circleCenter.x()+edgeDx*ratio, circleCenter.y()+edgeDy*ratio};
  const Vector circleTangentIntersectionPoint2{circleCenter.x()-edgeDx*ratio, circleCenter.y()-edgeDy*ratio};
  Vector edgePoint1, edgePoint2;
  double dist1 = distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleTangentIntersectionPoint1, &edgePoint1);
  double dist2 = distanceBetweenEdgeAndPoint(edgeStartPoint, edgeEndPoint, circleTangentIntersectionPoint2, &edgePoint2);
  double distanceResult;
  if (edgePoint1 == edgePoint2) {
    // Circle is beyond the extent of the line
    // Decide which distance to use
    if (crossProductForSign(edgePoint1, circleTangentIntersectionPoint1, edgePoint1, circleTangentIntersectionPoint2) > 0) {
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
    if (crossProductForSign(circleCenter, edgePoint1, circleCenter, edgePoint2) > 0) {
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
  theta = normalize(theta, k2Pi);
  if (equal(theta, 0.0) || equal(theta, kPi)) {
    return AngleDirection::kNoDirection;
  } else if (lessThan(theta, kPi)) {
    return AngleDirection::kCounterclockwise;
  } else {
    return AngleDirection::kClockwise;
  }
}


double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const Vector &point, const Vector &centerOfCircle, const double circleRadius) {
  // Find the angle between the intersection of tangent lines and the center of the circle
  double distanceToCircle = math::distance(point,centerOfCircle);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);
  return angleOfTangentLine;
}

std::pair<Vector, Vector> intersectionsPointsOfTangentLinesToCircle(const Vector &point, const Vector &centerOfCircle, const double circleRadius) {
  // Find the two lines that are tangent to the circle and intersect with the given point
  double distanceToCircle = math::distance(point,centerOfCircle);
  double lengthOfTangentLine = std::sqrt(distanceToCircle*distanceToCircle-circleRadius*circleRadius);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);

  // Calculate angle for center of circle
  double angleOfCenterOfCircle = math::angle(point, centerOfCircle);

  // First point is counterclockwise to circle
  // Second point is clockwise to circle
  double x1 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle+angleOfTangentLine);
  double y1 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle+angleOfTangentLine);

  double x2 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle-angleOfTangentLine);
  double y2 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle-angleOfTangentLine);
  return {Vector(x1, y1), Vector(x2, y2)};
}

double angle(const Vector &point1, const AngleDirection point1Direction, const Vector &point2, const AngleDirection point2Direction, const double circleRadius) {
  if (equal(point1, point2)) {
    throw std::runtime_error("Cannot find the angle betweeen a point and itself");
  }
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
    Vector midpoint;
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

  return normalize(angleBetweenPoints, k2Pi);
}

Vector extendLineSegmentToLength(const Vector &point1, const Vector &point2, const double targetLength) {
  const auto dx = point2.x()-point1.x();
  const auto dy = point2.y()-point1.y();
  const auto currentLength = sqrt(dx*dx+dy*dy);
  const auto ratio = targetLength / currentLength;
  return {point1.x() + dx*ratio, point1.y() + dy*ratio};
}

std::pair<Vector, Vector> createCircleConsciousLine(const Vector &point1, const AngleDirection &point1Direction, const Vector &point2, const AngleDirection &point2Direction, const double circleRadius) {
  constexpr const double kReducedPrecision{1e-3};
  if (math::equal(circleRadius, 0.0)) {
    return {point1, point2};
  }
  Vector lineStart, lineEnd;

  if (point1Direction == AngleDirection::kNoDirection) {
    lineStart = point1;
  } else {
    // point1 is a circle
    // Does point2 lie within the first circle?
    if (point2Direction != AngleDirection::kNoDirection) {
      // Both are circles, no potential issues
    } else {
      const auto distanceBetweenPoints = math::distance(point1, point2);
      if (math::equal(distanceBetweenPoints, circleRadius, kReducedPrecision)) {
        // point2 on the circumference of point1's circle
        // the line is really not a line
        if (math::equal(distanceBetweenPoints, circleRadius)) {
          return {point2, point2};
        } else {
          // There's a bit of a precision issue, lets use the point where this point would be on the circle's circumference
          const auto fixedPoint = extendLineSegmentToLength(point1, point2, circleRadius);
          return {fixedPoint, fixedPoint};
        }
      } else if (math::lessThan(distanceBetweenPoints, circleRadius)) {
        // TODO: point2 is inside point1's circle. Handle
        throw std::runtime_error("createCircleConsciousLine: point2 is inside point1's circle");
      }
    }
  }
  if (point2Direction == AngleDirection::kNoDirection) {
    lineEnd = point2;
  } else {
    // point2 is a circle
    // Does point1 lie within the first circle?
    if (point1Direction != AngleDirection::kNoDirection) {
      // Both are circles, no potential issues
    } else {
      const auto distanceBetweenPoints = math::distance(point1, point2);
      if (math::equal(distanceBetweenPoints, circleRadius, kReducedPrecision)) {
        // point1 on the circumference of point2's circle
        // the line is really not a line
        if (math::equal(distanceBetweenPoints, circleRadius)) {
          return {point1, point1};
        } else {
          // There's a bit of a precision issue, lets use the point where this point would be on the circle's circumference
          const auto fixedPoint = extendLineSegmentToLength(point2, point1, circleRadius);
          return {fixedPoint, fixedPoint};
        }
      } else if (math::lessThan(distanceBetweenPoints, circleRadius)) {
        // TODO: point1 is inside point2's circle. Handle
        throw std::runtime_error("createCircleConsciousLine: point1 is inside point2's circle");
      }
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
      double distanceToNewPoint = std::sqrt(distanceBetweenCircles*distanceBetweenCircles + circleRadius*circleRadius);
      double angleFromCircle2ToNewPoint = asin(circleRadius / distanceToNewPoint);
      double newAngle = absoluteAngleToCircle2 + angleFromCircle2ToNewPoint;

      double dxFromPoint1 = distanceToNewPoint * cos(newAngle);
      double dyFromPoint1 = distanceToNewPoint * sin(newAngle);

      Vector newPoint{point1.x() + dxFromPoint1, point1.y() + dyFromPoint1};
      double circleXOffset = newPoint.x() - point2.x();
      double circleYOffset = newPoint.y() - point2.y();

      if (point1Direction == AngleDirection::kClockwise) {
        // Both clockwise
        lineStart = Vector{point1.x() + circleXOffset, point1.y() + circleYOffset};
        lineEnd = Vector{point2.x() + circleXOffset, point2.y() + circleYOffset};
      } else {
        // Both counterclockwise
        lineStart = Vector{point1.x() - circleXOffset, point1.y() - circleYOffset};
        lineEnd = Vector{point2.x() - circleXOffset, point2.y() - circleYOffset};
      }
    } else {
      // Inner tangents
      // Find the point between these two circles
      Vector midpoint;
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

int lineSegmentIntersectsWithCircle(Vector lineSegmentStartPoint, Vector lineSegmentEndPoint, Vector centerOfCircle, const double circleRadius, Vector *intersectionPoint1, Vector *intersectionPoint2) {
  // First, shift the points over so that the circle is at the origin
  lineSegmentStartPoint.setX(lineSegmentStartPoint.x() - centerOfCircle.x());
  lineSegmentStartPoint.setY(lineSegmentStartPoint.y() - centerOfCircle.y());
  lineSegmentEndPoint.setX(lineSegmentEndPoint.x() - centerOfCircle.x());
  lineSegmentEndPoint.setY(lineSegmentEndPoint.y() - centerOfCircle.y());
  // ax -= cx;
  // ay -= cy;
  // bx -= cx;
  // by -= cy;

  // Now, calculate coefficients of quadratic equation
  const double dx = lineSegmentEndPoint.x() - lineSegmentStartPoint.x();
  const double dy = lineSegmentEndPoint.y() - lineSegmentStartPoint.y();
  const double a = dx*dx + dy*dy;
  const double b = 2 * (lineSegmentStartPoint.x()*dx + lineSegmentStartPoint.y()*dy);
  const double c = lineSegmentStartPoint.x()*lineSegmentStartPoint.x() + lineSegmentStartPoint.y()*lineSegmentStartPoint.y() - circleRadius*circleRadius;
  // a = (bx - ax)^2 + (by - ay)^2;
  // b = 2*(ax*(bx - ax) + ay*(by - ay));
  // c = ax^2 + ay^2 - r^2;

  // Now, calculate the discriminant
  //  Negative means no intersections
  //  0 means 1 intersection
  //  Positive means 2 intersections
  const double discriminant = b*b - 4*a*c;
  if (discriminant < 0) {
    return 0;
  }
  const double sqrtDiscriminant = sqrt(discriminant);
  // disc = b^2 - 4*a*c;
  // if(disc <= 0) return false;
  // sqrtdisc = sqrt(disc);

  // Finally, calculate the points of intersection
  double t1 = (-b + sqrtDiscriminant) / (2*a);
  double t2 = (-b - sqrtDiscriminant) / (2*a);
  if (discriminant > 0 && lessThan(t2, t1)) {
    // If there are two intersection points, make sure that the one determined by t1 is closer to the start of the line segment
    std::swap(t1, t2);
  }
  int intersectionCount{0};
  if (0 <= t1 && t1 <= 1) {
    if (intersectionPoint1 != nullptr) {
      intersectionPoint1->setX(lineSegmentStartPoint.x() + dx*t1 + centerOfCircle.x());
      intersectionPoint1->setY(lineSegmentStartPoint.y() + dy*t1 + centerOfCircle.y());
    }
    ++intersectionCount;
  }
  if ((discriminant > 0) && (0 <= t2 && t2 <= 1)) {
    Vector **point;
    if (intersectionCount == 0) {
      point = &intersectionPoint1;
    } else {
      point = &intersectionPoint2;
    }
    if ((*point) != nullptr) {
      (*point)->setX(lineSegmentStartPoint.x() + dx*t2 + centerOfCircle.x());
      (*point)->setY(lineSegmentStartPoint.y() + dy*t2 + centerOfCircle.y());
    }
    ++intersectionCount;
  }
  return intersectionCount;
  // t1 = (-b + sqrtdisc)/(2*a);
  // t2 = (-b - sqrtdisc)/(2*a);
  // if((0 < t1 && t1 < 1) || (0 < t2 && t2 < 1)) return true;
  // return false;
}

std::pair<Vector, Vector> createVectorTangentToPointOnCircle(const Vector &circleCenter, const double circleRadius, Vector pointOnCircleCircumference) {
  const double kReducedPrecision{1e-3};
  const auto distanceToPoint = distance(circleCenter, pointOnCircleCircumference);
  if (!equal(distanceToPoint, circleRadius)) {
    if (equal(distanceToPoint, circleRadius, kReducedPrecision)) {
      // Slight precision issue, move point to where it needs to be
      pointOnCircleCircumference = math::extendLineSegmentToLength(circleCenter, pointOnCircleCircumference, circleRadius);
    } else {
      throw std::runtime_error("createVectorTangentToPointOnCircle: Point is not on circumference of circle");
    }
  }
  if (equal(circleCenter.y(), pointOnCircleCircumference.y())) {
    // Point is on the exact left or right of circle. We know this is a vertical line
    return {{pointOnCircleCircumference.x(), pointOnCircleCircumference.y()-0.5},{pointOnCircleCircumference.x(), pointOnCircleCircumference.y()+0.5}};
  }
  const auto a = circleCenter.x();
  const auto b = circleCenter.y();
  const auto x1 = pointOnCircleCircumference.x();
  const auto y1 = pointOnCircleCircumference.y();
  const auto w = x1-a;
  const auto z = y1-b;
  const auto c = w*x1;
  const auto d = z*y1;

  Vector point1{-1, (d+c+w)/z};
  Vector point2{1, (d+c-w)/z};
  return {point1,point2};
}

double normalize(double in, double modVal) {
  if (modVal <= 0) {
    throw std::invalid_argument("pathfinder::math::normalize: modVal must be a positive number");
  }
  if (lessThan(in, 0) || !lessThan(in, modVal)) {
    return in - modVal*std::floor(in / modVal);
  } else {
    return in;
  }
}

IntersectionResult intersect(Vector line1V1, Vector line1V2, Vector line2V1, Vector line2V2, Vector *intersectionPoint) {
  const auto setIntersectionPoint = [&intersectionPoint](const Vector &point) {
    if (intersectionPoint != nullptr) {
      *intersectionPoint = point;
    }
  };

  const auto lessThan = [](const auto &p1, const auto &p2) {
    if (p1.x() == p2.x()) {
      return p1.y() < p2.y();
    } else {
      return p1.x() < p2.x();
    }
  };

  // Do a quick check for 0-length lines
  const bool line1IsZeroLength = (line1V1 == line1V2);
  const bool line2IsZeroLength = (line2V1 == line2V2);
  if (line1IsZeroLength && line2IsZeroLength) {
    if (line1V1 == line2V1) {
      // All 4 points are the same
      setIntersectionPoint(line1V1);
      return IntersectionResult::kOne;
    } else {
      // Two 0-length lines that do not touch
      return IntersectionResult::kNone;
    }
  } else if (line1IsZeroLength) {
    if (isPointOnLineSegment(line1V1, line2V1, line2V2)) {
      // Line 1 is just a point on line 2
      setIntersectionPoint(line1V1);
      return IntersectionResult::kOne;
    } else {
      // Line 1 does not touch line 2
      return IntersectionResult::kNone;
    }
  } else if (line2IsZeroLength) {
    if (isPointOnLineSegment(line2V1, line1V1, line1V2)) {
      // Line 2 is just a point on line 1
      setIntersectionPoint(line2V1);
      return IntersectionResult::kOne;
    } else {
      // Line 2 does not touch line 1
      return IntersectionResult::kNone;
    }
  }

  // Fact: Neither line is 0-length

  // Order points in each line
  if (lessThan(line1V2, line1V1)) {
    // Points are out of order
    std::swap(line1V1, line1V2);
  }
  if (lessThan(line2V2, line2V1)) {
    // Points are out of order
    std::swap(line2V1, line2V2);
  }

  // Line AB represented as a1x + b1y = c1
  const auto a1 = line1V2.y() - line1V1.y();
  const auto b1 = line1V1.x() - line1V2.x();
  const auto c1 = a1*line1V1.x() + b1*line1V1.y();

  // Line CD represented as a2x + b2y = c2
  const auto a2 = line2V2.y() - line2V1.y();
  const auto b2 = line2V1.x() - line2V2.x();
  const auto c2 = a2*line2V1.x()+ b2*line2V1.y();

  const auto determinant = a1*b2 - a2*b1;

  if (determinant == 0) {
    // Lines are parallel
    if (isPointOnLineSegment(line2V1, line1V1, line1V2)) {
      if (line2V1 == line1V2) {
        // Only share endpoints
        setIntersectionPoint(line2V1);
        return IntersectionResult::kOne;
      } else {
        // Since points are ordered, neither line is 0-length, and these lines are not sharing first/last endpoints, there must be infinite overlap
        return IntersectionResult::kInfinite;
      }
    }
    if (isPointOnLineSegment(line1V1, line2V1, line2V2)) {
      if (line1V1 == line2V2) {
        // Only share endpoints
        setIntersectionPoint(line1V1);
        return IntersectionResult::kOne;
      } else {
        // Since points are ordered, neither line is 0-length, and these lines are not sharing first/last endpoints, there must be infinite overlap
        return IntersectionResult::kInfinite;
      }
    }
    return IntersectionResult::kNone;
  } else {
    // Lines are not parallel
    const Vector intersection((c1*b2 - c2*b1) / determinant,
                              (a1*c2 - a2*c1) / determinant);
    if (isPointOnLineSegment(intersection, line1V1, line1V2) &&
        isPointOnLineSegment(intersection, line2V1, line2V2)) {
      // The intersection is on both of the line segments
      setIntersectionPoint(intersection);
      return IntersectionResult::kOne;
    } else {
      // Not on both line segments, no intersection
      return IntersectionResult::kNone;
    }
  }
}

bool trianglesOverlap(const Vector &triangle1V1, const Vector &triangle1V2, const Vector &triangle1V3, const Vector &triangle2V1, const Vector &triangle2V2, const Vector &triangle2V3, const bool onBoundary, const double tolerance) {
  const auto det2D = [](const auto &p1, const auto &p2, const auto &p3) {
    return p1.x()*(p2.y()-p3.y()) +
           p2.x()*(p3.y()-p1.y()) +
           p3.x()*(p1.y()-p2.y());
  };

  // TODO: Expose this to the user?
  constexpr const static bool kAllowReversed{true};

  const auto checkTriWinding = [&det2D](auto &p1, auto &p2, auto &p3) {
    const double detTri = det2D(p1, p2, p3);
    if(detTri < 0.0) {
      if constexpr (kAllowReversed) {
        std::swap(p2, p3);
      } else {
        throw std::runtime_error("Triangle has wrong winding direction");
      }
    }
  };

  const auto boundaryCollideCheck = [&det2D](const auto &p1, const auto &p2, const auto &p3, const double eps) {
    return det2D(p1, p2, p3) < eps;
  };

  const auto boundaryDoesntCollideCheck = [&det2D](const auto &p1, const auto &p2, const auto &p3, const double eps) {
    return det2D(p1, p2, p3) <= eps;
  };

  std::array<Vector,3> t1 = {triangle1V1, triangle1V2, triangle1V3},
                       t2 = {triangle2V1, triangle2V2, triangle2V3};

  // Trangles must be expressed anti-clockwise
  checkTriWinding(t1[0], t1[1], t1[2]);
  checkTriWinding(t2[0], t2[1], t2[2]);

  std::function<bool(const Vector&, const Vector&, const Vector&, const double)> chkEdge;
  if (onBoundary) {
    // Points on the boundary are considered as colliding
    chkEdge = boundaryCollideCheck;
  } else {
    // Points on the boundary are not considered as colliding
    chkEdge = boundaryDoesntCollideCheck;
  }

  // For edge E of trangle 1,
  for (int i=0; i<3; i++) {
    const int j=(i+1)%3;

    // Check all points of trangle 2 lay on the external side of the edge E. If
    // they do, the triangles do not collide.
    if (chkEdge(t1[i], t1[j], t2[0], tolerance) &&
        chkEdge(t1[i], t1[j], t2[1], tolerance) &&
        chkEdge(t1[i], t1[j], t2[2], tolerance)) {
      return false;
    }
  }

  // For edge E of trangle 2,
  for (int i=0; i<3; i++) {
    const int j=(i+1)%3;

    // Check all points of trangle 1 lay on the external side of the edge E. If
    // they do, the triangles do not collide.
    if (chkEdge(t2[i], t2[j], t1[0], tolerance) &&
        chkEdge(t2[i], t2[j], t1[1], tolerance) &&
        chkEdge(t2[i], t2[j], t1[2], tolerance)) {
      return false;
    }
  }

  // The triangles collide
  return true;
}

} // namespace math

} // namespace pathfinder
