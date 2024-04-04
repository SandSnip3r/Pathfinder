#include "path.h"

namespace pathfinder {

StraightPathSegment::StraightPathSegment(const Vector &start, const Vector &end) : startPoint(start), endPoint(end) {
}

StraightPathSegment::StraightPathSegment(const StraightPathSegment &other) : startPoint(other.startPoint), endPoint(other.endPoint) {
}

std::unique_ptr<PathSegment> StraightPathSegment::clone() const {
  return std::unique_ptr<PathSegment>(new StraightPathSegment(*this));
}

ArcPathSegment::ArcPathSegment(const Vector &center, const double radius, const AngleDirection direction) : circleCenter(center), circleRadius(radius), angleDirection(direction) {
}

ArcPathSegment::ArcPathSegment(const ArcPathSegment &other) : circleCenter(other.circleCenter), circleRadius(other.circleRadius), angleDirection(other.angleDirection), startAngle(other.startAngle), endAngle(other.endAngle) {
}

std::unique_ptr<PathSegment> ArcPathSegment::clone() const {
  return std::unique_ptr<PathSegment>(new ArcPathSegment(*this)); 
}

} // namespace pathfinder