#ifndef PATHFINDER_INTERVAL_H_
#define PATHFINDER_INTERVAL_H_

#include "math_helpers.h"
#include "vector.h"

#include <absl/log/log.h>
#include <absl/strings/str_format.h>

#include <optional>
#include <sstream>
#include <utility>

namespace pathfinder {

using LineSegment = std::pair<Vector, Vector>;

template<typename StateType, typename IndexType>
class Interval {
public:
  // TODO: Consolidate all of these setters into a constructor
  // Interval(const StateType &state,
  // const Vector &rootPoint, const std::optional<std::pair<IndexType, AngleDirection>> indexAndDirection,
  // const Vector &leftPoint, const std::optional<IndexType> leftIndex,
  // const Vector &rightPoint, const std::optional<IndexType> rightIndex,
  // double costToRoot) {
  //   if (leftIsRoot() && leftIndex && rootIndex && leftIndex != rootIndex) {
  //     // These are only possible when there's a shared vertex.
  //     throw std::runtime_error("Left is root, but left does not have the same index as the root");
  //   }
  //   if (rightIsRoot() && rightIndex && rootIndex && rightIndex != rootIndex) {
  //     // These are only possible when there's a shared vertex.
  //     throw std::runtime_error("Right is root, but right does not have the same index as the root");
  //   }
  // }
  Interval(const StateType &state) : state(state) {}
  void setRoot(const Vector &point) {
    rootPoint = point;
  }
  void setRoot(const Vector &point, std::optional<IndexType> index, AngleDirection direction) {
    rootPoint = point;
    rootIndex = index;
    rootDirection = direction;
  }
  void setLeft(const Vector &point, std::optional<IndexType> index = std::nullopt) {
    leftPoint = point;
    leftIndex = index;
  }
  void setRight(const Vector &point, std::optional<IndexType> index = std::nullopt) {
    rightPoint = point;
    rightIndex = index;
  }
  StateType state;
  Vector rootPoint, leftPoint, rightPoint;
  std::optional<IndexType> rootIndex, leftIndex, rightIndex;
  AngleDirection rootDirection{AngleDirection::kNoDirection};
  double costToRoot{0.0};
  bool isGoal{false};

  bool leftIsRoot() const {
    return rootPoint == leftPoint;
  }

  bool rightIsRoot() const {
    return rootPoint == rightPoint;
  }

  std::string toString() const {
    return absl::StrFormat("State{%s}, root{%s,%s,%.3f,%.3f}, left{%s,%.3f,%.3f}, right{%s,%.3f,%.3f}, costToRoot{%.5f}",
        state.toString(),
        pathfinder::toString(rootDirection), (rootIndex ? std::to_string(*rootIndex) : "_"), rootPoint.x(), rootPoint.y(),
        (leftIndex ? std::to_string(*leftIndex) : "_"), leftPoint.x(), leftPoint.y(),
        (rightIndex ? std::to_string(*rightIndex) : "_"), rightPoint.x(), rightPoint.y(),
        costToRoot);
  }

  // -------------------------------------------------------------------------------------------------------
  // Below are additional fields that get populated as we progress through the exploration of the interval.
  // -------------------------------------------------------------------------------------------------------
public:
  void setIntervalLeftIsConstraintVertex(bool isConstraint) {
    intervalLeftIsConstraintVertex_ = isConstraint;
    leftDirection_ = (*intervalLeftIsConstraintVertex_ ? AngleDirection::kCounterclockwise : AngleDirection::kNoDirection);
  }

  void setIntervalRightIsConstraintVertex(bool isConstraint) {
    intervalRightIsConstraintVertex_ = isConstraint;
    rightDirection_ = (*intervalRightIsConstraintVertex_ ? AngleDirection::kClockwise : AngleDirection::kNoDirection);
  }

  void setRightIntervals(const std::optional<LineSegment> &rightInterval, const std::optional<LineSegment> &rightIntervalToCurrentEntryEdge) {
    rightInterval_ = rightInterval;
    rightIntervalToCurrentEntryEdge_ = rightIntervalToCurrentEntryEdge;
  }

  void setLeftIntervals(const std::optional<LineSegment> &leftInterval, const std::optional<LineSegment> &leftIntervalToCurrentEntryEdge) {
    leftInterval_ = leftInterval;
    leftIntervalToCurrentEntryEdge_ = leftIntervalToCurrentEntryEdge;
  }

  bool intervalLeftIsConstraintVertex() const {
    if (!intervalLeftIsConstraintVertex_) {
      throw std::runtime_error("Getting value of intervalLeftIsConstraintVertex before assignment");
    }
    return intervalLeftIsConstraintVertex_.value();
  }

  bool intervalRightIsConstraintVertex() const {
    if (!intervalRightIsConstraintVertex_) {
      throw std::runtime_error("Getting value of intervalRightIsConstraintVertex before assignment");
    }
    return intervalRightIsConstraintVertex_.value();
  }

  AngleDirection leftDirection() const {
    if (!leftDirection_) {
      throw std::runtime_error("Getting value of leftDirection before assignment");
    }
    return leftDirection_.value();
  }

  AngleDirection rightDirection() const {
    if (!rightDirection_) {
      throw std::runtime_error("Getting value of rightDirection before assignment");
    }
    return rightDirection_.value();
  }

  const std::optional<LineSegment>& rightInterval() const {
    if (!rightInterval_) {
      throw std::runtime_error("Getting value of rightInterval before assignment");
    }
    return *rightInterval_;
  }

  const std::optional<LineSegment>& rightIntervalToCurrentEntryEdge() const {
    if (!rightIntervalToCurrentEntryEdge_) {
      throw std::runtime_error("Getting value of rightIntervalToCurrentEntryEdge before assignment");
    }
    return *rightIntervalToCurrentEntryEdge_;
  }

  const std::optional<LineSegment>& leftInterval() const {
    if (!leftInterval_) {
      throw std::runtime_error("Getting value of leftInterval before assignment");
    }
    return *leftInterval_;
  }

  const std::optional<LineSegment>& leftIntervalToCurrentEntryEdge() const {
    if (!leftIntervalToCurrentEntryEdge_) {
      throw std::runtime_error("Getting value of leftIntervalToCurrentEntryEdge before assignment");
    }
    return *leftIntervalToCurrentEntryEdge_;
  }
private:
  std::optional<bool> intervalLeftIsConstraintVertex_;
  std::optional<bool> intervalRightIsConstraintVertex_;
  std::optional<AngleDirection> leftDirection_;
  std::optional<AngleDirection> rightDirection_;
  std::optional<std::optional<LineSegment>> rightInterval_;
  std::optional<std::optional<LineSegment>> rightIntervalToCurrentEntryEdge_;
  std::optional<std::optional<LineSegment>> leftInterval_;
  std::optional<std::optional<LineSegment>> leftIntervalToCurrentEntryEdge_;
};

template<typename IntervalType>
class IntervalCompare {
public:
  bool operator()(const IntervalType &lhs, const IntervalType &rhs) const {
    auto pointIsLess = [](const auto &lp, const auto &rp) {
      if (lp.x() == rp.x()) {
        return lp.y() < rp.y();
      } else {
        return lp.x() < rp.x();
      }
    };

    // Ignore cost-to-root.
    if (lhs.isGoal == rhs.isGoal) {
      if (lhs.state == rhs.state) {
        if (lhs.rootDirection == rhs.rootDirection) {
          if (lhs.rootPoint == rhs.rootPoint) {
            if (lhs.leftPoint == rhs.leftPoint) {
              if (lhs.rightPoint == rhs.rightPoint) {
                if (lhs.rootIndex == rhs.rootIndex) {
                  if (lhs.leftIndex == rhs.leftIndex) {
                    return lhs.rightIndex < rhs.rightIndex;
                  } else {
                    return lhs.leftIndex < rhs.leftIndex;
                  }
                } else {
                  return lhs.rootIndex < rhs.rootIndex;
                }
              } else {
                return pointIsLess(lhs.rightPoint, rhs.rightPoint);
              }
            } else {
              return pointIsLess(lhs.leftPoint, rhs.leftPoint);
            }
          } else {
            return pointIsLess(lhs.rootPoint, rhs.rootPoint);
          }
        } else {
          return lhs.rootDirection < rhs.rootDirection;
        }
      } else {
        return lhs.state < rhs.state;
      }
    } else {
      return lhs.isGoal < rhs.isGoal;
    }
  }
};

} // namespace pathfinder

#endif // PATHFINDER_INTERVAL_H_