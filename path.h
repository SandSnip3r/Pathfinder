#ifndef PATHFINDER_PATH_H_
#define PATHFINDER_PATH_H_

#include "math_helpers.h"
#include "vector.h"

#include <memory>

namespace pathfinder {

struct PathSegment {
  virtual std::unique_ptr<PathSegment> clone() const = 0;
  virtual ~PathSegment();
};

struct StraightPathSegment : public PathSegment {
  StraightPathSegment(const Vector &start, const Vector &end);
  StraightPathSegment(const StraightPathSegment &other);
  std::unique_ptr<PathSegment> clone() const override;
  Vector startPoint, endPoint;
};

struct ArcPathSegment : public PathSegment {
  ArcPathSegment(const Vector &center, const double radius, const AngleDirection direction);
  ArcPathSegment(const ArcPathSegment &other);
  std::unique_ptr<PathSegment> clone() const override;
  Vector circleCenter;
  double circleRadius;
  AngleDirection angleDirection;
  double startAngle, endAngle;
};
  
} // namespace pathfinder

#endif // PATHFINDER_PATH_H_
