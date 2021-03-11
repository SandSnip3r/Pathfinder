#ifndef FUNNEL_H
#define FUNNEL_H

#include "math_helpers.h"

#include <QPointF>

#include <memory>
#include <optional>
#include <vector>

struct Apex {
  QPointF apexPoint;
  AngleDirection apexType;
};

class BaseFunnel;

class Funnel {
friend class BaseFunnel;
public:
  using iterator = std::vector<QPointF>::iterator;
  using const_iterator = std::vector<QPointF>::const_iterator;
  using reverse_iterator = std::vector<QPointF>::reverse_iterator;
  using const_reverse_iterator = std::vector<QPointF>::const_reverse_iterator;
  Funnel() = default; // TODO: Kind of in a weird state here
  Funnel(const QPointF &initialApex, const int corridorSize);
  int size() const;
  bool empty() const;
  const QPointF& at(int index) const;
  const QPointF& reverse_at(int index) const;
  const QPointF& front() const;
  void pop_front();
  void push_front(const QPointF &point);
  const QPointF& back() const;
  void pop_back();
  void push_back(const QPointF &point);
  bool point_in_funnel(const QPointF &point) const;
  void push_front_apex(const Apex &apex);
  void push_back_apex(const Apex &apex);
  Apex funnel_apex() const;
  const QPointF& apex_point() const;
  AngleDirection apex_type() const;
  int apex_index() const;
  int reverse_apex_index() const;
  void set_apex_index(const int index);
  void set_apex_type(const AngleDirection type);
  Funnel cloneButSpaceFor1More() const;
private:
  std::vector<QPointF> funnel_;
  AngleDirection apexType_{AngleDirection::kNoDirection};
  int apexIndex_;
  int leftIndex_, rightIndex_;
};

struct PathSegment {
  virtual std::unique_ptr<PathSegment> clone() const = 0;
  virtual ~PathSegment();
};

struct StraightPathSegment : public PathSegment {
  StraightPathSegment(const QPointF &start, const QPointF &end) : startPoint(start), endPoint(end) {}
  StraightPathSegment(const StraightPathSegment &other) : startPoint(other.startPoint), endPoint(other.endPoint) {}
  virtual std::unique_ptr<PathSegment> clone() const override { return std::unique_ptr<PathSegment>(new StraightPathSegment(*this)); }
  QPointF startPoint, endPoint;
};

struct ArcPathSegment : public PathSegment {
  ArcPathSegment(const QPointF &center, const double radius, const AngleDirection direction) : circleCenter(center), circleRadius(radius), angleDirection(direction) {}
  ArcPathSegment(const ArcPathSegment &other) : circleCenter(other.circleCenter), circleRadius(other.circleRadius), angleDirection(other.angleDirection), startAngle(other.startAngle), endAngle(other.endAngle) {}
  virtual std::unique_ptr<PathSegment> clone() const override { return std::unique_ptr<PathSegment>(new ArcPathSegment(*this)); }
  QPointF circleCenter;
  double circleRadius;
  AngleDirection angleDirection;
  double startAngle, endAngle;
};

class BaseFunnel {
public:
  BaseFunnel() = default; // TODO: Might not be worth having
  BaseFunnel(const double agentRadius);
  void funnelWithGoal(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint, const QPointF &goalPoint);
  void funnelWithoutGoal(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint);
  QPointF finishFunnelAndFindClosestGoalOnEdge(const std::pair<QPointF,QPointF> &edge);
  void finishFunnelWithGoal(const QPointF &goalPoint);
  void extendByOneEdge(const std::pair<QPointF,QPointF> &edge);
  const Funnel& getFunnel() const { return funnel_; } //TODO: Remove after done debugging

protected:
  double agentRadius_{0.0}; // TODO: Make constant
  Funnel funnel_;
  void addLeft(const QPointF &point, const bool isGoal=false);
  void addRight(const QPointF &point, const bool isGoal=false);
  void finishFunnel();

private:
  void initializeForFunnelAlgorithm(const int corridorSize, const QPointF &startPoint, const QPointF *goalPoint=nullptr);
  void funnelForCorridor(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint);
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) = 0;
  virtual double currentPathLength() const = 0;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const = 0;

  template<typename AtFunc, typename ApexIndexFunc, typename ApexTypeFunc, typename SetBeginningAsApexFunc, typename SetApexTypeFunc, typename PushFunc, typename PushApexFunc, typename PopFunc>
  void addPointToFunnel(const AtFunc &at,
                        const ApexIndexFunc &apex_index,
                        const ApexTypeFunc &getApexType,
                        const SetBeginningAsApexFunc &setBeginningAsApex,
                        const SetApexTypeFunc &setApexType,
                        const PushFunc &push,
                        const PushApexFunc &pushApex,
                        const PopFunc &pop,
                        const AngleDirection &direction,
                        const QPointF &point,
                        const bool isGoal);
};

class PathFunnel : public BaseFunnel {
public:
  using BaseFunnel::BaseFunnel;
  using PathType = std::vector<std::unique_ptr<PathSegment>>;
  PathType getPath() const;
private:
  PathType path_;
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const override;
};

class LengthFunnel : public BaseFunnel {
public:
  using BaseFunnel::BaseFunnel;
  LengthFunnel& operator=(const LengthFunnel &other) = default;
  double getLength() const;
  LengthFunnel cloneFunnelButSpaceFor1MorePoint() const;
private:
  double length_{0.0};
  std::optional<double> previousAngle_;
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const override;
  double funnelLengthForAgentWithRadius(LengthFunnel funnelCopy, const QPointF &goalPoint) const;
};

template<typename AtFunc, typename ApexIndexFunc, typename ApexTypeFunc, typename SetBeginningAsApexFunc, typename SetApexTypeFunc, typename PushFunc, typename PushApexFunc, typename PopFunc>
void BaseFunnel::addPointToFunnel(const AtFunc &at,
                                  const ApexIndexFunc &apex_index,
                                  const ApexTypeFunc &getApexType,
                                  const SetBeginningAsApexFunc &setBeginningAsApex,
                                  const SetApexTypeFunc &setApexType,
                                  const PushFunc &push,
                                  const PushApexFunc &pushApex,
                                  const PopFunc &pop,
                                  const AngleDirection &direction,
                                  const QPointF &point,
                                  const bool isGoal) {
  if (direction == AngleDirection::kNoDirection) {
    throw std::runtime_error("Direction \"kNoDirection\" does not make sense in this function");
  }

  const AngleDirection oppositeDirection = (direction == AngleDirection::kClockwise ? AngleDirection::kCounterclockwise : AngleDirection::kClockwise);
  // NOTE: Passing in Clockwise for 'direction' means "Right"
  auto directionString = [&direction]() -> std::string {
    if (direction == AngleDirection::kClockwise) {
      return "right";
    } else {
      return "left";
    }
  };

  auto rotatesInwardsViaCrossProduct = [&direction](const QPointF &v1Start, const QPointF &v1End, const QPointF &v2Start, const QPointF &v2End) {
    const double crossProductMagnitude = math::crossProduct(v1Start, v1End, v2Start, v2End);
    // Cross product should be <=0 for "Right" and > 0 for "Left"
    if (direction == AngleDirection::kClockwise) {
      return crossProductMagnitude <= 0;
    } else {
      return crossProductMagnitude > 0;
    }
  };

  auto newEdgeRotatesOutwards = [this, &at, &apex_index, &getApexType, &direction, &point, &isGoal]() {
    // Returns true if a potentially newly added edge would rotate toward the "inside" of the funnel
    // Note: We trust that there are at least 2 points and that the first is not the apex

    // std::cout << "Checking if a new edge (to point " << DebugLogger::instance().pointToString(point) << ") would rotate towards the inside of the funnel" << std::endl; //DEBUGPRINTS

    const auto &firstPoint = at(0);

    const QPointF &mostRecentEdgePoint1 = at(1);
    const QPointF &mostRecentEdgePoint2 = firstPoint;
    AngleDirection mostRecentEdgePoint1Direction;
    const AngleDirection mostRecentEdgePoint2Direction = direction;

    // We know that the first point is not the apex, but the second point could be
    if (apex_index() == 1) {
      mostRecentEdgePoint1Direction = getApexType();
    } else {
      // Neither points are the apex
      mostRecentEdgePoint1Direction = direction;
    }

    const QPointF &newEdgePoint1 = firstPoint;
    const QPointF &newEdgePoint2 = point;
    const AngleDirection newEdgePoint1Direction = direction;
    const AngleDirection newEdgePoint2Direction = (isGoal ? AngleDirection::kNoDirection : direction);

    const double mostRecentEdgeAngle = math::angle(mostRecentEdgePoint1, mostRecentEdgePoint1Direction, mostRecentEdgePoint2, mostRecentEdgePoint2Direction, agentRadius_);
    const double newEdgeAngle = math::angle(newEdgePoint1, newEdgePoint1Direction, newEdgePoint2, newEdgePoint2Direction, agentRadius_);
    const double newEdgeRelativeAngle = newEdgeAngle - mostRecentEdgeAngle;
    return (math::angleRelativeToOrigin(newEdgeRelativeAngle) == direction);
  };
  // std::cout << "  << Adding " << DebugLogger::instance().pointToString(point) << " to " << directionString() << " of funnel" << std::endl; //DEBUGPRINTS

  std::optional<Apex> newApex;
  // Make sure there is at least one edge in the funnel
  if (funnel_.size() >= 2) {
    // Remove edges that are more outward-rotating than this new potential edges
    while (apex_index() != 0 && !newEdgeRotatesOutwards()) {
      // std::cout << "    << Popping from " << directionString() << " of funnel" << std::endl; //DEBUGPRINTS
      pop();
    }

    // Need to check if this new edge would cross over the apex
    while (apex_index() == 0 && funnel_.size() >= 2) {
      const auto &currentApexPoint = at(0);
      std::pair<QPointF, QPointF> newWedge = math::createCircleConsciousLine(currentApexPoint, getApexType(), point, (isGoal ? AngleDirection::kNoDirection : direction), agentRadius_);
      std::pair<QPointF, QPointF> firstEdgeInOtherDirection = math::createCircleConsciousLine(currentApexPoint, getApexType(), at(1), oppositeDirection, agentRadius_);
      if (rotatesInwardsViaCrossProduct(newWedge.first, newWedge.second, firstEdgeInOtherDirection.first, firstEdgeInOtherDirection.second)) {
        // New point crosses over apex
        // std::cout << "    << New point crosses over apex" << std::endl; //DEBUGPRINTS
        if (math::distance(newWedge.first, newWedge.second) < math::distance(firstEdgeInOtherDirection.first, firstEdgeInOtherDirection.second)) {
          // New point is closer and should instead be the apex
          // std::cout << "      << New point is closer and should instead be the apex" << std::endl; //DEBUGPRINTS

          newApex = Apex{point, (isGoal ? AngleDirection::kNoDirection : direction)};

          addSegment(funnel_.funnel_apex(), newWedge, *newApex);

          // Remove old apex
          pop();

          // Apex is now a point not yet in the funnel, this loop is done
          break;
        } else {
          // std::cout << "      << Normal add of apex to path" << std::endl; //DEBUGPRINTS
          // Add the apex to the path
          Apex updatedApex{at(1), oppositeDirection};

          addSegment(funnel_.funnel_apex(), firstEdgeInOtherDirection, updatedApex);

          // Remove old apex
          pop();

          // First point is now the current apex, update
          setBeginningAsApex();
          setApexType(updatedApex.apexType);
        }
      } else {
        // Done
        // std::cout << "    << Done" << std::endl; //DEBUGPRINTS
        break;
      }
    }
  }
  // Finally, add point to beginning of funnel
  if (newApex) {
    // Add new apex to funnel
    // std::cout << "  << Adding point to " << directionString() << " of funnel, as apex" << std::endl; //DEBUGPRINTS
    pushApex(*newApex);
  } else {
    // Only adding normal point to funnel
    // std::cout << "  << Adding point to " << directionString() << " of funnel" << std::endl; //DEBUGPRINTS
    push(point);
  }
}

#endif // FUNNEL_H