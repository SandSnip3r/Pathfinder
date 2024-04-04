#ifndef PATHFINDER_FUNNEL_H_
#define PATHFINDER_FUNNEL_H_

#include "debuglogger.h"
#include "math_helpers.h"
#include "path.h"
#include "vector.h"

#include <iostream>

#include <memory>
#include <optional>
#include <vector>

namespace pathfinder {

struct Apex {
  Vector apexPoint;
  AngleDirection apexType;
};

class BaseFunnel;

class Funnel {
friend class BaseFunnel;
public:
  Funnel() = default; // TODO: Kind of in a weird state here
  Funnel(const Vector &initialApex, const std::size_t corridorSize);
  int size() const;
  bool empty() const;
  const Vector& at(int index) const;
  const Vector& reverse_at(int index) const;
  const Vector& front() const;
  void pop_front();
  void push_front(const Vector &point);
  const Vector& back() const;
  void pop_back();
  void push_back(const Vector &point);
  bool point_in_funnel(const Vector &point) const;
  void push_front_apex(const Apex &apex);
  void push_back_apex(const Apex &apex);
  Apex funnel_apex() const;
  const Vector& apex_point() const;
  AngleDirection apex_type() const;
  int apex_index() const;
  int reverse_apex_index() const;
  void set_apex_index(const int index);
  void set_apex_type(const AngleDirection type);
  Funnel cloneButSpaceFor1More() const;
private:
  std::vector<Vector> funnel_;
  AngleDirection apexType_{AngleDirection::kNoDirection};
  int apexIndex_;
  int leftIndex_, rightIndex_;
};

class BaseFunnel {
public:
  BaseFunnel() = default; // TODO: Might not be worth having
  BaseFunnel(const double agentRadius);
  void funnelWithGoal(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint, const Vector &goalPoint);
  void funnelWithoutGoal(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint);
  Vector finishFunnelAndFindClosestGoalOnEdge(const std::pair<Vector,Vector> &edge);
  void finishFunnelWithGoal(const Vector &goalPoint);
  void extendByOneEdge(const std::pair<Vector,Vector> &edge);
  const Funnel& getFunnel() const { return funnel_; } //TODO: Remove after done debugging

protected:
  double agentRadius_{0.0}; // TODO: Make constant
  Funnel funnel_;
  bool pointInFunnel(const Vector &point) const;
  void addLeft(const Vector &point, const bool isGoal=false);
  void addRight(const Vector &point, const bool isGoal=false);
  void finishFunnel();
  virtual void addSegment(const Apex &previousApex, const std::pair<Vector,Vector> &edge, const Apex &newApex) = 0;

private:
  void initializeForFunnelAlgorithm(const std::size_t corridorSize, const Vector &startPoint, const Vector *goalPoint=nullptr);
  void funnelForCorridor(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint);
  virtual double currentPathLength() const = 0;
  virtual Vector findBestGoalForFunnel(const std::pair<Vector,Vector> &lastEdgeOfCorridor) const = 0;

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
                        const Vector &point,
                        const bool isGoal);
};

class PathFunnel : public BaseFunnel {
public:
  using BaseFunnel::BaseFunnel;
  using PathType = std::vector<std::unique_ptr<PathSegment>>;
  PathType getPath() const;
private:
  PathType path_;
  virtual void addSegment(const Apex &previousApex, const std::pair<Vector,Vector> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual Vector findBestGoalForFunnel(const std::pair<Vector,Vector> &lastEdgeOfCorridor) const override;
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
  virtual void addSegment(const Apex &previousApex, const std::pair<Vector,Vector> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual Vector findBestGoalForFunnel(const std::pair<Vector,Vector> &lastEdgeOfCorridor) const override;
  double funnelLengthForAgentWithRadius(LengthFunnel funnelCopy, const Vector &goalPoint) const;
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
                                  const Vector &point,
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

  enum class CrossOverResult {
    kCrossesOver,
    kDoesNotCross,
    kLinesAreParallel
  };

  auto newEdgeCrossesOverApexAndAdvancesFunnel = [&direction](const Vector &v1Start, const Vector &v1End, const Vector &v2Start, const Vector &v2End) {
    if (math::equal(v1Start, v1End) || math::equal(v2Start, v2End)) {
      throw std::runtime_error("newEdgeCrossesOverApexAndAdvancesFunnel: Cannot determine angle of 0-length vector");
    }
    auto normalizeIfNecessary = [](const auto &p1, const auto &p2) {
      if (math::lessThan(math::distanceSquared(p1, p2), 0.1)) {
        // Short vector, normalize to length 1
        return math::extendLineSegmentToLength(p1, p2, 1.0);
      } else {
        return p2;
      }
    };
    // For the cross product magnitude calculation, make sure the vector should is long enough to avoid precision issues
    const Vector normalizedV1End = normalizeIfNecessary(v1Start, v1End);
    const Vector normalizedV2End = normalizeIfNecessary(v2Start, v2End);
    const double crossProductMagnitude = math::crossProductForSign(v1Start, normalizedV1End, v2Start, normalizedV2End);
    // Cross product should be <=0 for "Right" and > 0 for "Left"
    //  This could happen in a few high level cases
    //  Possible case 1: The agent is trying to pass through and edge that is exactly his diameter. In this case, the edge would be trimmed down to a single point and the two given vectors would be identical
    //    Imagine a closed funnel where left and right are touching
    //  Possible case 2: The agent is trying to pass through and edge that he starts on (or that the goal is on)
    //    Imagine a funnel that is completely open @ 180 degrees
    if (math::equal(crossProductMagnitude, 0)) {
      const auto angleOfVector1 = math::angle(v1Start, normalizedV1End);
      const auto angleOfVector2 = math::angle(v2Start, normalizedV2End);
      if (math::equal(angleOfVector1, angleOfVector2)) {
        // Vectors are the same direction, this essentialy means that the funnel is closed up to point v2End
        return CrossOverResult::kLinesAreParallel;
      } else if (math::equal(math::normalize(angleOfVector1-angleOfVector2, math::k2Pi), math::kPi)) {
        // Vectors are opposite direction, the funnel is wide open
        return CrossOverResult::kDoesNotCross;
      }
      throw std::runtime_error("newEdgeCrossesOverApexAndAdvancesFunnel: Cross product is 0, angles are not the same nor opposites");
    }
    const bool crosses = [&direction, &crossProductMagnitude]() {
      if (direction == AngleDirection::kClockwise) {
        return math::lessThan(crossProductMagnitude, 0);
      } else {
        return math::lessThan(0, crossProductMagnitude);
      }
    }();
    if (crosses) {
      return CrossOverResult::kCrossesOver;
    } else {
      return CrossOverResult::kDoesNotCross;
    }
  };

  auto newEdgeRotatesOutwards = [this, &at, &apex_index, &getApexType, &direction, &point, &isGoal]() {
    // Returns true if a potentially newly added edge would rotate toward the "inside" of the funnel
    // Note: We trust that there are at least 2 points and that the first is not the apex

    // std::cout << "newEdgeRotatesOutwards:: Checking if a new edge (to point " << DebugLogger::instance().pointToString(point) << ") would rotate away from the inside of the funnel" << std::endl; //DEBUGPRINTS

    const auto &firstPoint = at(0);

    const Vector &mostRecentEdgePoint1 = at(1);
    const Vector &mostRecentEdgePoint2 = firstPoint;
    AngleDirection mostRecentEdgePoint1Direction;
    const AngleDirection mostRecentEdgePoint2Direction = direction;
    // std::cout << "newEdgeRotatesOutwards:: Most recent edge: " << DebugLogger::instance().pointToString(mostRecentEdgePoint1) << " -> " << DebugLogger::instance().pointToString(mostRecentEdgePoint2) << std::endl; //DEBUGPRINTS

    // We know that the first point is not the apex, but the second point could be
    if (apex_index() == 1) {
      mostRecentEdgePoint1Direction = getApexType();
    } else {
      // Neither points are the apex
      mostRecentEdgePoint1Direction = direction;
    }

    const Vector &newEdgePoint1 = firstPoint;
    const Vector &newEdgePoint2 = point;
    const AngleDirection newEdgePoint1Direction = direction;
    const AngleDirection newEdgePoint2Direction = (isGoal ? AngleDirection::kNoDirection : direction);

    const double mostRecentEdgeAngle = math::angle(mostRecentEdgePoint1, mostRecentEdgePoint1Direction, mostRecentEdgePoint2, mostRecentEdgePoint2Direction, agentRadius_);
    const double newEdgeAngle = math::angle(newEdgePoint1, newEdgePoint1Direction, newEdgePoint2, newEdgePoint2Direction, agentRadius_);
    const double newEdgeRelativeAngle = newEdgeAngle - mostRecentEdgeAngle;
    if (math::equal(newEdgeRelativeAngle, 0.0)) {
      // These vectors are oriented in the same angle
      // std::cout << "newEdgeRotatesOutwards:: it would not" << std::endl; //DEBUGPRINTS
      return false;
    }
    // std::cout << "newEdgeRotatesOutwards:: it would " << ((math::angleRelativeToOrigin(newEdgeRelativeAngle) != direction) ? "not" : "") << std::endl; //DEBUGPRINTS
    return (math::angleRelativeToOrigin(newEdgeRelativeAngle) == direction);
  };

  auto checkIfMostRecentPointIsFartherFromApexThanFirstPointOnOtherSideOfApex = [this, &at, &apex_index]() {
    // This function checks if the distance from the apex to the point most recently added on this side of the funnel
    //  is farther away than the first point on the other side of the apex
    const auto apexIndex = apex_index();
    if (funnel_.size() <= apexIndex+1) {
      // Not enough points to do this check, cant be true
      return false;
    }
    const Vector &apexPoint = at(apexIndex);
    const Vector &mostRecentEdgePoint1 = apexPoint;
    const Vector &mostRecentEdgePoint2 = at(0);
    const Vector &firstEdgeInOtherDirectionPoint1 = apexPoint;
    const Vector &firstEdgeInOtherDirectionPoint2 = at(apexIndex+1);
    return math::distance(mostRecentEdgePoint1, mostRecentEdgePoint2) > math::distance(firstEdgeInOtherDirectionPoint1, firstEdgeInOtherDirectionPoint2);
  };
  // std::cout << "  << Adding " << DebugLogger::instance().pointToString(point) << " to " << directionString() << " of funnel" << std::endl; //DEBUGPRINTS
  // std::cout << "  << Current funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS

  std::optional<Apex> newApex;
  bool poppedVertexThatWasFarther{false};
  // Make sure there is at least one edge in the funnel
  if (funnel_.size() >= 2) {
    // Remove edges that are more outward-rotating than this new potential edges
    while (apex_index() != 0 && !newEdgeRotatesOutwards()) {
      // std::cout << "    << Popping from " << directionString() << " of funnel" << std::endl; //DEBUGPRINTS
      poppedVertexThatWasFarther = poppedVertexThatWasFarther || checkIfMostRecentPointIsFartherFromApexThanFirstPointOnOtherSideOfApex();
      // If we end up popping a vertex that is farther away than the first point on the other side of the apex,
      //  we dont want to conclude later that this new point is actually closer than the first point on the other side of the apex.
      //  I am not convinced that this is a perfect solution in general, but I believe that the constraints of the triangulation
      //  make this test meaningful
      // if (poppedVertexThatWasFarther) { //DEBUGPRINTS
      //   std::cout << "    << This point that we popped was farther from the apex than the first edge in the other direction, this information will be useful later" << std::endl; //DEBUGPRINTS
      // } //DEBUGPRINTS
      pop();
      // std::cout << "    << Post-pop funnel: "; //DEBUGPRINTS
      // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
    }

    // Need to check if this new edge would cross over the apex
    while (apex_index() == 0 && funnel_.size() >= 2) {
      const auto &currentApexPoint = at(0);
      // std::cout << "newWedge is " << DebugLogger::instance().pointToString(currentApexPoint) << ' ' << (getApexType() == AngleDirection::kNoDirection ? " point" : (getApexType() == AngleDirection::kClockwise ? " right" : " left")) << " -> " //DEBUGPRINTS
                                  // << DebugLogger::instance().pointToString(point)            << ' ' << (isGoal ? "point" : directionString()) << std::endl; //DEBUGPRINTS
      // std::cout << "firstEdgeInOtherDirection is " //DEBUGPRINTS
          // << DebugLogger::instance().pointToString(currentApexPoint) << ' ' << (getApexType() == AngleDirection::kNoDirection ? " point" : (getApexType() == AngleDirection::kClockwise ? " right" : " left")) << " -> " //DEBUGPRINTS
          // << DebugLogger::instance().pointToString(at(1))            << ' ' << (oppositeDirection == AngleDirection::kNoDirection ? " point" : (oppositeDirection == AngleDirection::kClockwise ? " right" : " left")) << std::endl; //DEBUGPRINTS
      std::pair<Vector, Vector> newWedge = math::createCircleConsciousLine(currentApexPoint, getApexType(), point, (isGoal ? AngleDirection::kNoDirection : direction), agentRadius_);
      std::pair<Vector, Vector> firstEdgeInOtherDirection = math::createCircleConsciousLine(currentApexPoint, getApexType(), at(1), oppositeDirection, agentRadius_);
      std::pair<Vector, Vector> newWedgeForAngleComparison = newWedge;
      std::pair<Vector, Vector> firstEdgeInOtherDirectionForAngleComparison = firstEdgeInOtherDirection;
      // std::cout << "Comparing newWedge " << newWedge.first.x() << ',' << newWedge.first.y() << "->" << newWedge.second.x() << ',' << newWedge.second.y() << " to firstEdgeInOtherDirection " << firstEdgeInOtherDirection.first.x() << ',' << firstEdgeInOtherDirection.first.y() << "->" << firstEdgeInOtherDirection.second.x() << ',' << firstEdgeInOtherDirection.second.y() << std::endl; //DEBUGPRINTS
      bool madeFakeVectorForNewWedge{false};
      if (math::equal(math::distance(newWedge.first, newWedge.second), 0)) {
        // newWedge is length 0
        bool newPointIsPointlike = (isGoal ? true : (direction == AngleDirection::kNoDirection));
        Vector pointForTangentVector;
        if (getApexType() == AngleDirection::kNoDirection && newPointIsPointlike) {
          throw std::runtime_error("newWedge: Both points are points");
        } else if (getApexType() != AngleDirection::kNoDirection && !newPointIsPointlike) {
          // Both points are circles
          pointForTangentVector = newWedge.first;
        } else {
          pointForTangentVector = currentApexPoint;
        }
        // This is a point that is on a circle around `point`
        // This does happen in cases other than the start and the goal

        // Create a tangent line to the circle in order to do an angle test
        auto tangentVector = math::createVectorTangentToPointOnCircle(point, agentRadius_, pointForTangentVector);
        // Flip vector if orientation is incorrect
        // We expect that tangentVector.first is the start and tangentVector.second is the end
        const auto crossProductResult = math::crossProductForSign(point, tangentVector.first, point, tangentVector.second);
        if ((direction == AngleDirection::kClockwise && crossProductResult > 0) ||
            (direction == AngleDirection::kCounterclockwise && crossProductResult < 0)) {
          // Incorrect orientation
          std::swap(tangentVector.first, tangentVector.second);
        }
        newWedgeForAngleComparison = tangentVector;
        madeFakeVectorForNewWedge = true;
      }
      bool madeFakeVectorForOtherWedge{false};
      if (math::equal(math::distance(firstEdgeInOtherDirection.first, firstEdgeInOtherDirection.second), 0)) {
        // firstEdgeInOtherDirection is length 0
        Vector pointOnCircle;
        if (getApexType() == AngleDirection::kNoDirection && oppositeDirection == AngleDirection::kNoDirection) {
          throw std::runtime_error("firstEdgeInOtherDirection: Both points are points");
        } else if (getApexType() != AngleDirection::kNoDirection && oppositeDirection != AngleDirection::kNoDirection) {
          // This happens when going through an edge that is exactly our width
          pointOnCircle = firstEdgeInOtherDirection.first;
        } else {
          // This is a point that is on a circle around `at(1)`
          // TODO: Can this really ever happen in a case besides the start or goal? (i think so)
          pointOnCircle = currentApexPoint;
        }
        // Create a tangent line to the circle in order to do an angle test
        auto tangentVector = math::createVectorTangentToPointOnCircle(at(1), agentRadius_, pointOnCircle);

        // Flip vector if orientation is incorrect
        // We expect that tangentVector.first is the start and tangentVector.second is the end
        const auto crossProductResult = math::crossProductForSign(at(1), tangentVector.first, at(1), tangentVector.second);
        if ((oppositeDirection == AngleDirection::kClockwise && crossProductResult > 0) ||
            (oppositeDirection == AngleDirection::kCounterclockwise && crossProductResult < 0)) {
          // Incorrect orientation
          std::swap(tangentVector.first, tangentVector.second);
        }
        firstEdgeInOtherDirectionForAngleComparison = tangentVector;
        madeFakeVectorForOtherWedge = true;
      }
      const auto crossOverResult = newEdgeCrossesOverApexAndAdvancesFunnel(newWedgeForAngleComparison.first, newWedgeForAngleComparison.second, firstEdgeInOtherDirectionForAngleComparison.first, firstEdgeInOtherDirectionForAngleComparison.second);
      if (crossOverResult != CrossOverResult::kDoesNotCross) {
        // New point might cross over apex
        // std::cout << "    << New point might cross over apex" << std::endl; //DEBUGPRINTS
        if (!poppedVertexThatWasFarther && (!madeFakeVectorForOtherWedge && !madeFakeVectorForNewWedge) && math::distance(newWedge.first, newWedge.second) < math::distance(firstEdgeInOtherDirection.first, firstEdgeInOtherDirection.second)) {
          // New point is closer and should instead be the apex
          // std::cout << "      << New point is closer and should instead be the apex" << std::endl; //DEBUGPRINTS

          newApex = Apex{point, (isGoal ? AngleDirection::kNoDirection : direction)};

          addSegment(funnel_.funnel_apex(), newWedge, *newApex);

          // Remove old apex
          pop();

          // Apex is now a point not yet in the funnel, this loop is done
          break;
        } else {
          if (madeFakeVectorForOtherWedge) {
            // std::cout << "      << New point crosses over 0-length other wedge segment" << std::endl; //DEBUGPRINTS
            // In this case, it does not make sense to create a new apex when the two lines are parallel
            // TODO: Evaluate other cases
            // Only known case:
            //  Passing through an edge which a non-zero-width agent can only perfectly fit through
            if (crossOverResult == CrossOverResult::kLinesAreParallel) {
              break;
            }
          } else if (madeFakeVectorForNewWedge) {
            // std::cout << "      << New point crosses over 0-length new wedge segment" << std::endl; //DEBUGPRINTS
          } else {
            // std::cout << "      << Normal add of apex to path" << std::endl; //DEBUGPRINTS
          }
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
      // std::cout << "  << This iteration is done, funnel: "; //DEBUGPRINTS
      // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
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

} // namespace pathfinder

#endif // PATHFINDER_FUNNEL_H_
