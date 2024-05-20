#ifndef PATHFINDER_PATHFINDER_H_
#define PATHFINDER_PATHFINDER_H_

// #include "binary_min_heap.h"
#include "debuglogger.h"
#include "interval.h"
#include "math_helpers.h"
#include "path.h"
#include "vector.h"

#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <absl/strings/str_format.h>

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

namespace pathfinder {

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path);

namespace internal {

enum class RelativePositionToInterval {
  kIsLeftOf,
  kIsRightOf,
  kIsWithin
};

std::string toString(RelativePositionToInterval pos);
bool lineActuallyIntersectedWithCircle(const pathfinder::Vector &lineSegmentStartPoint,
                                       const pathfinder::Vector &lineSegmentEndPoint,
                                       const pathfinder::Vector &centerOfCircle,
                                       const double circleRadius,
                                       int intersectionCount,
                                       const pathfinder::Vector &intersectionPoint1,
                                       const pathfinder::Vector &intersectionPoint2);

struct RAIIPrinter {
  RAIIPrinter(const std::string &msg);
  ~RAIIPrinter();
  private:
  std::string msg_;
};

} // namespace internal

/* TODO: Outline NavmeshType requirements
 *
 *
 */

template<typename NavmeshType>
class Pathfinder {
public:
  static constexpr bool kProduceDebugAnimationData_{false};
  using State = typename NavmeshType::State;
  using IndexType = typename NavmeshType::IndexType;
  using IntervalType = Interval<State, IndexType>;

  struct IntervalAndCost {
    IntervalAndCost(const IntervalType &i, double c) : interval(i), cost(c) {}
    IntervalType interval;
    double cost;
  };
  struct IntervalAndCostComp {
    bool operator()(const IntervalAndCost &lhs, const IntervalAndCost &rhs) const {
      return lhs.cost > rhs.cost;
    }
  };
  using IntervalHeapType = std::priority_queue<IntervalAndCost, std::vector<IntervalAndCost>, IntervalAndCostComp>;
  using IntervalCompareType = pathfinder::IntervalCompare<IntervalType>;
  using PreviousIntervalMapType = std::unordered_map<IntervalType, IntervalType>;
  using IntervalSetType = std::unordered_set<IntervalType>;

  struct PathfindingAStarInfo {
    enum class PushOrPop {
      kPush, kPop
    };
    std::vector<std::pair<Pathfinder::IntervalType, PushOrPop>> intervals;
    PreviousIntervalMapType previous;
  };

  struct PathfindingResult {
    std::vector<std::unique_ptr<PathSegment>> shortestPath;
    struct EmptyType {};
    using DebugAStarInfoType = std::conditional_t<kProduceDebugAnimationData_, PathfindingAStarInfo, EmptyType>;
    DebugAStarInfoType debugAStarInfo;
  };

  Pathfinder(const NavmeshType &navmesh, const double agentRadius);
  void setTimeout(std::chrono::milliseconds timeoutMilliseconds);

  static constexpr bool hasDebugAnimationData() {
    return kProduceDebugAnimationData_;
  }

  // template<typename PointType>
  // std::pair<Vector, IndexType> getPointAwayFromConstraint(const PointType &point) const;

  template<typename PointType>
  PathfindingResult findShortestPath(const PointType &startPoint, const PointType &goalPoint) const;
private:
  const NavmeshType &navmesh_;
  const double agentRadius_;
  std::optional<std::chrono::milliseconds> timeoutMilliseconds_;

  mutable std::unordered_map<State, std::unordered_map<IndexType, bool>> constraintForStateCache_;

  // Does a cheaper check (compared to actual pathfinding) to see if it is possible at all to get from the start state to the goal state.
  bool canGetToState(const State &startState, const State &goalState, const std::chrono::high_resolution_clock::time_point pathfindingStartTime) const;

  PathfindingResult polyanya(const Vector &startPoint, const State &startState, const Vector &goalPoint, const State &goalState, const std::chrono::high_resolution_clock::time_point pathfindingStartTime) const;

  // Helpers
  std::tuple<Vector, IndexType, Vector, IndexType> getLeftAndRight(const State &successorState) const;
  bool isAConstraintVertexForState(const State &currentState, const IndexType vertexIndex) const;

  using BendResult = std::tuple<std::optional<Vector>, std::optional<IndexType>, std::optional<std::pair<Vector, IndexType>>>;
  // Returns the splitting point as projectedPoint, projectedIndex
  // Returns the rightConstraint
  BendResult bendThatMfRoundTheRight(const Vector &successorEdgeRightPoint,
                                     const IndexType successorEdgeRightIndex,
                                     const IntervalType &currentInterval,
                                     const Vector &successorEdgeLeftPoint,
                                     const IndexType successorEdgeLeftIndex,
                                     const double kEndpointIntersectionPrecision,
                                     const State &currentState) const;
  // Returns the splitting point as projectedPoint, projectedIndex
  // Returns the leftConstraint
  BendResult bendThatMfRoundTheLeft(const Vector &successorEdgeRightPoint,
                                    const IndexType successorEdgeRightIndex,
                                    const IntervalType &currentInterval,
                                    const Vector &successorEdgeLeftPoint,
                                    const IndexType successorEdgeLeftIndex,
                                    const double kEndpointIntersectionPrecision,
                                    const State &currentState) const;

  std::pair<Vector,Vector> createTempRightIntervalForCrossProduct(const IntervalType &currentInterval, const Vector &point) const;
  std::pair<Vector,Vector> createTempLeftIntervalForCrossProduct(const IntervalType &currentInterval, const Vector &point) const;

  // Given an `IntervalType` and line segment (toDestinationStart->toDestinationEnd), determine the line's relation to the left and right of the interval.
  std::pair<bool,bool> booberGobblin(const std::optional<IndexType> successorEdgeLeftIndex,
                                     const std::optional<IndexType> successorEdgeRightIndex,
                                     const IntervalType &currentInterval,
                                     const Vector &toDestinationStart,
                                     const Vector &toDestinationEnd) const;

  double costFromIntervalToGoal(const IntervalType &interval, const Vector &goalPoint) const;
  void pushSuccessor(const IntervalType *currentInterval,
                     IntervalType successorInterval,
                     IntervalHeapType &intervalHeap,
                     IntervalSetType &visited,
                     IntervalSetType &pushed,
                     PreviousIntervalMapType &previous,
                     const Vector &goalPoint,
                     typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const;
  void handleStartStateSuccessor(const State &successorState,
                                 const Vector &startPoint,
                                 const State &startState,
                                 const Vector &goalPoint,
                                 IntervalHeapType &intervalHeap,
                                 IntervalSetType &visited,
                                 IntervalSetType &pushed,
                                 PreviousIntervalMapType &previous,
                                 typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const;
  void expandInterval(IntervalType &currentInterval,
                      const Vector &startPoint,
                      const Vector &goalPoint,
                      const State &goalState,
                      IntervalHeapType &intervalHeap,
                      IntervalSetType &visited,
                      IntervalSetType &pushed,
                      PreviousIntervalMapType &previous,
                      typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const;
  void checkLeftAndRightConstraints(IntervalType &currentInterval) const;
  bool successorIsInsideIntervalConstraint(const IntervalType &currentInterval, const State &successorState) const;
  void buildResultFromGoalInterval(const IntervalType &goalInterval,
                                   const Vector &startPoint,
                                   const Vector &goalPoint,
                                   PreviousIntervalMapType &previous,
                                   PathfindingResult &result) const;
  void handleNormalSuccessor(const State &currentState,
                             const State &successorState,
                             const IntervalType &currentInterval,
                             const Vector &goalPoint,
                             IntervalHeapType &intervalHeap,
                             IntervalSetType &visited,
                             IntervalSetType &pushed,
                             PreviousIntervalMapType &previous,
                             typename PathfindingResult::DebugAStarInfoType &debugAStarInfo
                             ) const;
  void buildLeftIntervals(IntervalType &currentInterval) const;
  void buildRightIntervals(IntervalType &currentInterval) const;
  void handleGoalSuccessor(const IntervalType &currentInterval,
                           const State &successorState,
                           IntervalHeapType &intervalHeap,
                           IntervalSetType &visited,
                           IntervalSetType &pushed,
                           PreviousIntervalMapType &previous,
                           const Vector &goalPoint,
                           typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const;
  bool doesRightIntervalIntersectWithLeftOfSuccessorEdge(const IntervalType &currentInterval,
                                                         bool successorLeftIsConstraintVertex,
                                                         IndexType successorEdgeLeftIndex,
                                                         const Vector &successorEdgeLeftPoint,
                                                         const Vector &successorEdgeRightPoint) const;
  bool doesLeftIntervalIntersectWithRightOfSuccessorEdge(const IntervalType &currentInterval,
                                                         bool successorRightIsConstraintVertex,
                                                         IndexType successorEdgeRightIndex,
                                                         const Vector &successorEdgeLeftPoint,
                                                         const Vector &successorEdgeRightPoint) const;

  internal::RelativePositionToInterval calculateSuccessorLeftRelativeToInterval(const IntervalType &currentInterval,
                                                                 IndexType currentEdgeLeftIndex,
                                                                 const Vector &successorEdgeLeftPoint,
                                                                 IndexType successorEdgeLeftIndex,
                                                                 const Vector &successorEdgeRightPoint,
                                                                 IndexType successorEdgeRightIndex,
                                                                 bool successorLeftIsConstraintVertex) const;
  internal::RelativePositionToInterval calculateSuccessorRightRelativeToInterval(const IntervalType &currentInterval,
                                                                  IndexType currentEdgeRightIndex,
                                                                  const Vector &successorEdgeLeftPoint,
                                                                  IndexType successorEdgeLeftIndex,
                                                                  const Vector &successorEdgeRightPoint,
                                                                  IndexType successorEdgeRightIndex,
                                                                  bool successorRightIsConstraintVertex) const;
  bool intervalIsClosed(const IntervalType &currentInterval,
                        const Vector &successorEdgeLeftPoint,
                        const Vector &successorEdgeRightPoint,
                        bool checkIfIntersectWithLeftConstraint,
                        bool checkIfIntersectWithRightConstraint) const;
  bool canFitThroughEdge(const State &currentState, IndexType edgeIndex) const;


  // Checks if the given point collides with any of the constraint vertices of the triangle
  // If the distance to a constraint vertex is exactly the agent radius, then it will not collide
  // TODO: Does not check if the given point collides with edges of the triangle
  // std::optional<Vector> getCollidingConstraint(const Vector &point, const IndexType triangleIndex) const;

  // void buildShortestPathWithinSingleTriangle(const IndexType triangleIndex, const Vector &startPoint, const Vector &goalPoint, std::vector<std::unique_ptr<PathSegment>> &shortestPath) const;

  // double distanceBetweenEdgeAndPoint(IndexType edgeIndex, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr) const;
  // double calculateArcLength(const IndexType edge1Index, const IndexType edge2Index) const;
  // double calculateHValue(const State &state, const Vector &goalPoint) const;
  // double calculateEstimateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, double> &gScores) const;
  // std::tuple<double, Vector, std::optional<LengthFunnel>> calculateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, State> &previous) const;
  // PathfindingAStarInfo triangleAStar(const Vector &startPoint, const State &startState, const Vector &goalPoint, const State &goalState) const;

  // bool pathCanExist(const State &startState, const State &goalState) const;
  // std::vector<std::pair<Vector,Vector>> buildCorridor(const std::vector<IndexType> &trianglesInCorridor) const;

  // static std::vector<IndexType> rebuildPath(State state, const std::unordered_map<State, State> &previous);
};

template<typename NavmeshType>
Pathfinder<NavmeshType>::Pathfinder(const NavmeshType &navmesh, const double agentRadius) : navmesh_(navmesh), agentRadius_(agentRadius) {
  // Initialize debug logger
  DebugLogger::instance().setPointToIndexFunction(std::bind(&NavmeshType::getVertexIndex, std::cref(navmesh_), std::placeholders::_1));
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::setTimeout(std::chrono::milliseconds timeoutMilliseconds) {
  timeoutMilliseconds_ = timeoutMilliseconds;
}

// template<typename NavmeshType>
// template<typename PointType>
// std::pair<Vector, typename Pathfinder<NavmeshType>::IndexType> Pathfinder<NavmeshType>::getPointAwayFromConstraint(const PointType &point) const {
//   Vector point2d = NavmeshType::to2dPoint(point);
//   std::optional<IndexType> triangleForPoint = navmesh_.findTriangleForPoint(point2d);
//   if (!triangleForPoint) {
//     throw std::runtime_error("Unable to find valid triangle for point");
//   }
//   constexpr const int kMaxIterations{5};
//   int iterationCount{0};
//   std::optional<Vector> collidingConstraint = getCollidingConstraint(point2d, *triangleForPoint);
//   while (collidingConstraint.has_value()) {
//     if (iterationCount == kMaxIterations) {
//       throw std::runtime_error("Cannot point away from constraint");
//     }

//     // Push point out to radius of circle.
//     point2d = math::extendLineSegmentToLength(*collidingConstraint, point2d, agentRadius_*1.01);
//     triangleForPoint = navmesh_.findTriangleForPoint(point2d);
//     if (!triangleForPoint) {
//       throw std::runtime_error("Unable to find valid triangle for point");
//     }

//     // Check if the new point overlaps with any constraint.
//     collidingConstraint = getCollidingConstraint(point2d, *triangleForPoint);

//     // Keep track that we dont get stuck here for too long.
//     ++iterationCount;
//   }
//   return {point2d, *triangleForPoint};
// }

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::isAConstraintVertexForState(const State &currentState, const typename NavmeshType::IndexType vertexIndex) const {
  // TODO: Instead of a cache, this could be computed as a pre-computation step.
  auto &subMap = constraintForStateCache_[currentState];
  if (const auto it = subMap.find(vertexIndex); it != subMap.end()) {
    return it->second;
  }
  // Check if we can do a 360 around this vertex without running into any constraints.
  std::queue<State> nextStates;
  std::set<State> visited;
  nextStates.push(currentState);
  while (!nextStates.empty()) {
    auto currentState = nextStates.front();
    nextStates.pop();
    visited.emplace(currentState);
    const auto successorStates = navmesh_.getSuccessors(currentState, std::nullopt, 0.0);
    // Do any of these successors go through an edge which has the vertex in question as an endpoint?
    for (const auto &successorState : successorStates) {
      if (!successorState.hasEntryEdgeIndex()) {
        // Has no entry edge. What does that mean?
        continue;
      }
      const auto [v1, v2] = navmesh_.getEdgeVertexIndices(successorState.getEntryEdgeIndex());
      if (v1 != vertexIndex && v2 != vertexIndex) {
        // This successor is across an edge that does not touch our vertex; we don't care about this successor.
        continue;
      }
      if (visited.find(successorState) != visited.end()) {
        // We've already visited this state! That means we made a full loop!
        subMap.emplace(vertexIndex, false);
        return false;
      }
      // We have not yet visited this state.
      nextStates.push(successorState);
    }
  }
  subMap.emplace(vertexIndex, true);
  return true;
}

template<typename NavmeshType>
template<typename PointType>
typename Pathfinder<NavmeshType>::PathfindingResult Pathfinder<NavmeshType>::findShortestPath(const PointType &startPoint, const PointType &goalPoint) const {
  const auto pathfindingStartTime = std::chrono::high_resolution_clock::now();
  // { // TODO: <Remove>
  //   // // Print specific vertices.
  //   // const std::vector<IndexType> indicesToPrint = {
  //   //   486,487,488,603,601,520,521,522,613,583,577,477,483,482,480,479,478,476,481,489,485,600,599,598,602,604
  //   // };
  //   // for (const auto index : indicesToPrint) {
  //   //   if (index >= navmesh_.getVertexCount()) {
  //   //     VLOG(1) << "Asking for a vertex which is out of bounds!";
  //   //     continue;
  //   //   }
  //   //   const auto &vertex = navmesh_.getVertex(index);
  //   //   VLOG(1) << absl::StreamFormat("Index %d is (%.15f,%.15f)", index, vertex.x(), vertex.y());
  //   // }
  //   // ------------------------------------------------------------------------
  //   // Print vertices within a bounding box.
  //   constexpr double kMinX = 1015.0;
  //   constexpr double kMaxX = 1070.0;
  //   constexpr double kMinY = 1430.0;
  //   constexpr double kMaxY = 1480.0;
  //   std::vector<Vector> vertices;
  //   for (size_t i=0; i<navmesh_.getVertexCount(); ++i) {
  //     const auto &vertex = navmesh_.getVertex(i);
  //     if (vertex.x() >= kMinX && vertex.x() <= kMaxX &&
  //         vertex.y() >= kMinY && vertex.y() <= kMaxY) {
  //       vertices.push_back(vertex);
  //     }
  //   }
  //   VLOG(1) << vertices.size() << " 2 0 0";
  //   for (size_t i=0; i<vertices.size(); ++i) {
  //     VLOG(1) << absl::StreamFormat("%d %.20f %.20f", i, vertices[i].x(), vertices[i].y());
  //   }
  // } // </Remove>

  // Find the triangle for the start point.
  Vector startPoint2d = NavmeshType::to2dPoint(startPoint);
  std::optional<IndexType> startTriangle = navmesh_.findTriangleForPoint(startPoint2d);
  if (!startTriangle) {
    throw std::runtime_error("Unable to find start triangle");
  }
  const auto startState = navmesh_.createStartState(startPoint, *startTriangle);

  // Find the triangle for the goal point.
  Vector goalPoint2d = NavmeshType::to2dPoint(goalPoint);
  std::optional<IndexType> goalTriangle = navmesh_.findTriangleForPoint(goalPoint2d);
  if (!goalTriangle) {
    throw std::runtime_error("Unable to find goal triangle");
  }
  const auto goalState = navmesh_.createGoalState(goalPoint, *goalTriangle);

  // Do a quick check to see if it's even possible to get from the start triangle to the goal triangle.
  bool isMaybePossible = canGetToState(startState, goalState, pathfindingStartTime);
  if (!isMaybePossible) {
    // For sure no way to get there.
    return {};
  }

  return polyanya(startPoint2d, startState, goalPoint2d, goalState, pathfindingStartTime);
}

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::canGetToState(const State &startState, const State &goalState, const std::chrono::high_resolution_clock::time_point pathfindingStartTime) const {
  struct StateAndCost {
    StateAndCost(const State &i, double d) : state(i), distanceToGoal(d) {}
    State state;
    double distanceToGoal;
  };

  struct StateAndCostComp {
    bool operator()(const StateAndCost &lhs, const StateAndCost &rhs) const {
      return lhs.distanceToGoal > rhs.distanceToGoal;
    }
  };

  auto centerOfTriangleOfState = [this](const State &state) -> Vector {
    const auto& [v1,v2,v3] = navmesh_.getTriangleVertices(state.getTriangleIndex());
    return (v1 + v2 + v3) / 3.0;
  };
  std::priority_queue<StateAndCost, std::vector<StateAndCost>, StateAndCostComp> stateHeap;
  std::set<State> visited;

  // Calculate the center of the goal triangle for rough distance calculations.
  const auto centerOfGoal = centerOfTriangleOfState(goalState);

  auto pushState = [&centerOfTriangleOfState, &centerOfGoal](auto &stateHeap, const State &state) {
    const auto centerOfState = centerOfTriangleOfState(state);
    stateHeap.emplace(state, math::distance(centerOfState, centerOfGoal));
  };

  pushState(stateHeap, startState);

  while (!stateHeap.empty()) {
    if (timeoutMilliseconds_) {
      if (std::chrono::high_resolution_clock::now() >= pathfindingStartTime+*timeoutMilliseconds_) {
        const auto diff = std::chrono::high_resolution_clock::now() - pathfindingStartTime;
        LOG(INFO) << "Pathfinder timed out after " << std::chrono::duration_cast<std::chrono::microseconds>(diff).count() << "us";
        return {};
      }
    }
    const auto stateAndCost = stateHeap.top();
    const State &currentState = stateAndCost.state;;
    stateHeap.pop();

    // Check if this is the goal.
    if (currentState.isGoal()) {
      return true;
    }

    visited.emplace(currentState);
    auto successorStates = navmesh_.getSuccessors(currentState, goalState, 0.0); // TODO: Change 0.0 to agentRadius_
    for (const auto &state : successorStates) {
      if (visited.find(state) == visited.end()) {
        pushState(stateHeap, state);
      }
    }
  }
  // Couldn't get to goal.
  return false;
}

template<typename NavmeshType>
typename Pathfinder<NavmeshType>::PathfindingResult Pathfinder<NavmeshType>::polyanya(const Vector &startPoint, const State &startState, const Vector &goalPoint, const State &goalState, const std::chrono::high_resolution_clock::time_point pathfindingStartTime) const {
  PathfindingResult result;
  typename PathfindingResult::DebugAStarInfoType &debugAStarInfo = result.debugAStarInfo;

  if (startState.isSameTriangleAs(goalState)) {
    // Start and goal is in same triangle. Path is a straight line to the goal.
    // TODO: This is not always safe.
    result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(startPoint, goalPoint)));
    return result;
  }

  IntervalHeapType intervalHeap;
  PreviousIntervalMapType previous;
  IntervalSetType visited;
  IntervalSetType pushed;
  VLOG(1) << "Starting with state "  << startPoint << " in triangle " << startState.getTriangleIndex() << " with " << (startState.hasEntryEdgeIndex() ? "entry edge "+std::to_string(startState.getEntryEdgeIndex()) : "no entry edge");

  // Initialize the list of intervals.
  const auto startStateSuccessors = navmesh_.getSuccessors(startState, goalState, agentRadius_);
  for (const auto &successorState : startStateSuccessors) {
    handleStartStateSuccessor(successorState, startPoint, startState, goalPoint, intervalHeap, visited, pushed, previous, debugAStarInfo);
  }

  // Run the A* algorithm as defined by the Polyanya paper.
  while (!intervalHeap.empty()) {
    if (timeoutMilliseconds_) {
      if (std::chrono::high_resolution_clock::now() >= pathfindingStartTime+*timeoutMilliseconds_) {
        const auto diff = std::chrono::high_resolution_clock::now() - pathfindingStartTime;
        LOG(INFO) << "Pathfinder timed out after " << std::chrono::duration_cast<std::chrono::microseconds>(diff).count() << "us";
        return {};
      }
    }
    VLOG(1) << "\n===== Next iteration of A* =====";

    if (VLOG_IS_ON(1)) {
      // Print all intervals.
      // There's no way to iterate my heap in order, so we copy it and continually pop and print the top element.
      IntervalHeapType copyOfHeap = intervalHeap;
      while (!copyOfHeap.empty()) {
        const IntervalAndCost top = copyOfHeap.top();
        copyOfHeap.pop();
        VLOG(1) << "    Interval " << top.interval.toString() << " has cost " << top.cost;
      }
    }

    // Pop the next interval off the minheap and expand it.
    IntervalAndCost intervalWithCost = intervalHeap.top();
    intervalHeap.pop();
    IntervalType &currentInterval = intervalWithCost.interval;
    VLOG(1) << "The minimum element is " << currentInterval.toString() << " with cost " << intervalWithCost.cost;

    visited.emplace(currentInterval);
    if constexpr (kProduceDebugAnimationData_) {
      debugAStarInfo.intervals.emplace_back(currentInterval, PathfindingAStarInfo::PushOrPop::kPop);
    }

    if (currentInterval.isGoal) {
      buildResultFromGoalInterval(currentInterval, startPoint, goalPoint, previous, result);
      break;
    }

    expandInterval(currentInterval, startPoint, goalPoint, goalState, intervalHeap, visited, pushed, previous, debugAStarInfo);
  }
  if constexpr (kProduceDebugAnimationData_) {
    debugAStarInfo.previous = previous;
  }
  return result;
}

template<typename NavmeshType>
typename Pathfinder<NavmeshType>::BendResult Pathfinder<NavmeshType>::bendThatMfRoundTheRight(
          const Vector &successorEdgeRightPoint,
          const IndexType successorEdgeRightIndex,
          const IntervalType &currentInterval,
          const Vector &successorEdgeLeftPoint,
          const IndexType successorEdgeLeftIndex,
          const double kEndpointIntersectionPrecision,
          const State &currentState) const {
  if (VLOG_IS_ON(1)) {
    std::cout << std::endl;
  }
  VLOG(1) << "Enter bendThatMfRoundTheRight";
  internal::RAIIPrinter raiiPrinter("Exiting bendThatMfRoundTheRight\n");
  Vector projectedRightPoint;
  std::optional<IndexType> projectedRightIndex;
  bool successorEdgeIntersectsWithRightOfInterval=false;
  if (currentInterval.rightIsRoot()) {
    // We can definitely get to the right vertex of the successor edge.
    projectedRightPoint = successorEdgeRightPoint;
    projectedRightIndex = successorEdgeRightIndex;
  } else {
    if (!currentInterval.rightInterval()) {
      throw std::runtime_error("Right is not root and do not have a right interval");
    }
    const LineSegment &rightInterval = *currentInterval.rightInterval();
    bool createdProjectedPoint{false};
    if (currentInterval.intervalRightIsConstraintVertex() && *currentInterval.rightIndex != successorEdgeRightIndex) {
      VLOG(1) << "Right of interval is constraint and the right of the edge is a different point";
      // Check if the successor edge intersects with the right of the interval.
      Vector intersectionPoint1, intersectionPoint2;
      int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeLeftPoint, successorEdgeRightPoint, currentInterval.rightPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
      VLOG(1) << "Checking if segment Segment(" << '(' << successorEdgeLeftPoint.x() << ',' << successorEdgeLeftPoint.y() << ')' << ',' << '(' << successorEdgeRightPoint.x() << ',' << successorEdgeRightPoint.y() << ')' << ") intersects with point at " << currentInterval.rightPoint.x() << ',' << currentInterval.rightPoint.y();
      if (intersectionCount > 0) {
        // Only use this intersection if it is not inside the right point of the successor edge (if that point is a constraint).
        VLOG(1) << "[bendThatMfRoundTheRight] Itersection count is " << intersectionCount;
        if (!isAConstraintVertexForState(currentState, successorEdgeRightIndex) || math::distanceSquared(intersectionPoint1, successorEdgeRightPoint) > agentRadius_*agentRadius_) {
          successorEdgeIntersectsWithRightOfInterval = true;
          // One interval will be
          //  root: same as current
          //  left: successorEdgeLeftPoint, successorEdgeLeftIndex
          //  right: currentInterval.right
          projectedRightPoint = currentInterval.rightPoint;
          projectedRightIndex = currentInterval.rightIndex;
          createdProjectedPoint = true;

          if (intersectionCount == 2) {
            VLOG(1) << "Oh. hmm";
            // TODO:
            // If there are two intersection points, there will be another interval which is
            //  root: currentInterval.right
            //  left: successorEdgeRightPoint, successorEdgeRightIndex
            //  right: currentInterval.right
            // throw std::runtime_error("Still need to handle");
          }
        }
      }
    } else {
      VLOG(1) << "Right is not a constraint";
    }
    {
      double t1, t2;
      auto ir = math::intersectForIntervals(rightInterval.first, rightInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      if (ir != math::IntersectionResult::kInfinite) {
        const Vector point = successorEdgeLeftPoint + (successorEdgeRightPoint-successorEdgeLeftPoint)*t2;
        VLOG(1) << "Intersected at " << absl::StreamFormat("%.20f,%.20f", point.x(), point.y());
      } else {
        VLOG(1) << "Inf intersection";
      }
    }

    if (!createdProjectedPoint) {
      VLOG(1) << "Havent yet created a projected point";
      math::IntersectionResult intersectionResult;
      double t1, t2;
      if (math::equal(rightInterval.first, rightInterval.second)) {
        VLOG(1) << absl::StreamFormat("Right interval has length %.20f", math::distance(rightInterval.first, rightInterval.second));
        // Right interval has 0-length, create a fake temporary vector for an intersection test.
        Vector tangentStart, tangentEnd;
        if (!currentInterval.intervalRightIsConstraintVertex()) {
          // How do we create a temporary right interval if the right isnt a constraint?
          // Is there a root point and is this right interval on it?
          if (currentInterval.rootIndex && math::equal(math::distanceSquared(currentInterval.rootPoint, rightInterval.first), agentRadius_*agentRadius_)) {
            std::tie(tangentStart, tangentEnd) = math::createVectorTangentToPointOnCircle(currentInterval.rootPoint, agentRadius_, rightInterval.first);
          } else {
            VLOG(1) << "currentInterval.rootIndex: " << (bool)currentInterval.rootIndex;
            VLOG(1) << absl::StreamFormat("Distance squared from %.5f,%.5f to %.5f,%.5f is %.10f", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), rightInterval.first.x(), rightInterval.first.y(), math::distanceSquared(currentInterval.rootPoint, rightInterval.first));
            throw std::runtime_error("Unseen case 1");
          }
        } else {
          std::tie(tangentStart, tangentEnd) = math::createVectorTangentToPointOnCircle(currentInterval.rightPoint, agentRadius_, rightInterval.first);
          VLOG(1) << "Created tmp interval as Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ")";
        }
        intersectionResult = math::intersectForIntervals(tangentStart, tangentEnd, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      } else {
        intersectionResult = math::intersectForIntervals(rightInterval.first, rightInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      }

      if (math::equal(t2, 0.0, kEndpointIntersectionPrecision)) {
        // Right point is actually the left of the edge.
        projectedRightPoint = successorEdgeLeftPoint;
        projectedRightIndex = successorEdgeLeftIndex;
        VLOG(1) << "Projection is on left endpoint";
      } else if (math::equal(t2, 1.0, kEndpointIntersectionPrecision)) {
        // Right point is actually the right of the edge.
        projectedRightPoint = successorEdgeRightPoint;
        projectedRightIndex = successorEdgeRightIndex;
        VLOG(1) << "Projection is on right endpoint";
      } else {
        projectedRightPoint = successorEdgeLeftPoint + t2 * (successorEdgeRightPoint - successorEdgeLeftPoint);
        VLOG(1) << "Projection is at " << projectedRightPoint.x() << ',' << projectedRightPoint.y();
      }
    }

    if (!currentInterval.intervalRightIsConstraintVertex()) {
      // Right of interval is not a constraint, but the interval could intersect with a constraint on the way to the successor edge.
      // Extend right interval out to successor.
      VLOG(1) << "Checking if right interval intersects with a constraint on the way to the successor edge";
      double i1, i2;
      auto intersectionResult = math::intersectForIntervals(rightInterval.first, rightInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &i1, &i2);
      if (!math::lessThan(i2, 0.0) && !math::lessThan(1.0, i2)) {
        VLOG(1) << "This does intersect with the successor edge";
        const Vector rightIntervalIntersectionWithSuccessorEdge = successorEdgeLeftPoint + (successorEdgeRightPoint - successorEdgeLeftPoint) * i2;
        // For now, we only check if this intersects with the currrent entry edge's right.
        // TODO: Check if this line intersects with any other constraint vertices within range.
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        if (isAConstraintVertexForState(currentState, currentEdgeRightIndex)) {
          VLOG(1) << "Right of current edge is a constraint";
          Vector intersectionPoint1, intersectionPoint2;
          auto intersectionResult2 = math::lineSegmentIntersectsWithCircle(rightInterval.first, rightIntervalIntersectionWithSuccessorEdge, currentEdgeRightPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
          // Weed out the tangential intersections.
          const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, rightIntervalIntersectionWithSuccessorEdge, currentEdgeRightPoint, agentRadius_, intersectionResult2, intersectionPoint1, intersectionPoint2);
          if (actuallyIntersected) {
            VLOG(1) << "Augmented right interval Segment(" << '(' << rightInterval.first.x() << ',' << rightInterval.first.y() << "),(" << rightIntervalIntersectionWithSuccessorEdge.x() << ',' << rightIntervalIntersectionWithSuccessorEdge.y() << ')' << ") intersects with right of current edge point " << currentEdgeRightIndex;
            return {currentEdgeRightPoint, currentEdgeRightIndex, std::nullopt};
          }
        }
      }
    }
  }

  // The left of the interval might not be a vertex, but the left interval extended out to the next edge might pass through the left vertex of the successor edge.
  std::optional<Vector> optionalProjectedRightPoint = projectedRightPoint;
  std::optional<std::pair<Vector, IndexType>> rightConstraint;
  if (!successorEdgeIntersectsWithRightOfInterval) {
    VLOG(1) << "Successor edge does not intersect with right of interval";
    if (!currentInterval.rightIsRoot() && isAConstraintVertexForState(currentState, successorEdgeRightIndex) && math::lineSegmentIntersectsWithCircle(currentInterval.rightInterval()->first, projectedRightPoint, successorEdgeRightPoint, agentRadius_) == 2) {
      VLOG(1) << "Right interval to projected point intersects with successor right edge twice";
      // We will need to create two intervals. First create a tangent line to the right point of the successor edge. One interval will go straight to the projected point of that tangent. The other interval will bend around the right of that successor edge.
      const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rightInterval()->first, AngleDirection::kNoDirection, successorEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
      double t1_2, t2_2;
      const Vector pushedSuccessorRight = math::extendLineSegmentToLength(successorEdgeRightPoint, successorEdgeLeftPoint, agentRadius_);
      const Vector pushedSuccessorLeft = isAConstraintVertexForState(currentState, successorEdgeLeftIndex) ? math::extendLineSegmentToLength(successorEdgeLeftPoint, successorEdgeRightPoint, agentRadius_) : successorEdgeLeftPoint;
      auto intersectionResult2 = math::intersectForIntervals(lineStart, lineEnd, pushedSuccessorLeft, pushedSuccessorRight, &t1_2, &t2_2);
      // TODO: This line might not actually make it to the successor edge before hitting a constraint.
      VLOG(1) << "Intersection interval 1: " << t1_2 << ", 2: " << t2_2;
      (void)intersectionResult2;
      if (math::betweenOrEqual(t2_2, 0.0, 1.0)) {
        // Update projected right point
        optionalProjectedRightPoint = lineStart + t1_2 * (lineEnd - lineStart);
        VLOG(1) << "Created projected point is at " << optionalProjectedRightPoint->x() << ',' << optionalProjectedRightPoint->y();
      } else {
        VLOG(1) << "Didnt actually hit successor edge. Not going to return a projected point.";
        optionalProjectedRightPoint.reset();
      }
      rightConstraint.emplace(successorEdgeRightPoint, successorEdgeRightIndex);
    } else if (currentInterval.rightIndex && isAConstraintVertexForState(currentState, *currentInterval.rightIndex)) {
      rightConstraint.emplace(currentInterval.rightPoint, *currentInterval.rightIndex);
    }
  }
  return {optionalProjectedRightPoint, projectedRightIndex, rightConstraint};
}

template<typename NavmeshType>
typename Pathfinder<NavmeshType>::BendResult Pathfinder<NavmeshType>::bendThatMfRoundTheLeft(
          const Vector &successorEdgeRightPoint,
          const IndexType successorEdgeRightIndex,
          const IntervalType &currentInterval,
          const Vector &successorEdgeLeftPoint,
          const IndexType successorEdgeLeftIndex,
          const double kEndpointIntersectionPrecision,
          const State &currentState) const {
  if (VLOG_IS_ON(1)) {
    std::cout << std::endl;
  }
  VLOG(1) << "Enter bendThatMfRoundTheLeft";
  internal::RAIIPrinter raiiPrinter("Exiting bendThatMfRoundTheLeft\n");
  Vector projectedLeftPoint;
  std::optional<IndexType> projectedLeftIndex;
  bool successorEdgeIntersectsWithLeftOfInterval=false;
  if (currentInterval.leftIsRoot()) {
    // We can definitely get to the left vertex of the successor edge.
    projectedLeftPoint = successorEdgeLeftPoint;
    projectedLeftIndex = successorEdgeLeftIndex;
  } else {
    if (!currentInterval.leftInterval()) {
      throw std::runtime_error("Should not get here without left interval");
    }
    const LineSegment &leftInterval = *currentInterval.leftInterval();
    bool createdProjectedPoint{false};
    if (currentInterval.intervalLeftIsConstraintVertex() && *currentInterval.leftIndex != successorEdgeLeftIndex) {
      VLOG(1) << "Left of interval is constraint and the left of the edge is a different point";
      // Check if the successor edge intersects with the left of the interval.
      Vector intersectionPoint1, intersectionPoint2;
      int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeLeftPoint, successorEdgeRightPoint, currentInterval.leftPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
      VLOG(1) << "Checking if segment Segment(" << '(' << successorEdgeLeftPoint.x() << ',' << successorEdgeLeftPoint.y() << ')' << ',' << '(' << successorEdgeRightPoint.x() << ',' << successorEdgeRightPoint.y() << ')' << ") intersects with point at " << currentInterval.leftPoint.x() << ',' << currentInterval.leftPoint.y();
      if (intersectionCount > 0) {
        // Only use this intersection if it is not inside the right point of the successor edge (if that point is a constraint).
        VLOG(1) << "[bendThatMfRoundTheLeft] Itersection count is " << intersectionCount;
        if (!isAConstraintVertexForState(currentState, successorEdgeLeftIndex) || math::distanceSquared(intersectionPoint1, successorEdgeLeftPoint) > agentRadius_*agentRadius_) {
          successorEdgeIntersectsWithLeftOfInterval = true;
          // One interval will be
          //  root: same as current
          //  left: currentInterval.left
          //  right: successorEdgeRightPoint, successorEdgeRightIndex
          projectedLeftPoint = currentInterval.leftPoint;
          projectedLeftIndex = currentInterval.leftIndex;
          createdProjectedPoint = true;

          if (intersectionCount == 2) {
            VLOG(1) << "Oh. hmm";
            // TODO:
            // If there are two intersection points, there will be another interval which is
            //  root: currentInterval.left
            //  left: currentInterval.left
            //  right: successorEdgeLeftPoint, successorEdgeLeftIndex
            // throw std::runtime_error("Still need to handle");
          }
        }
      }
    } else {
      VLOG(1) << "Left is not a constraint";
    }
    if (!createdProjectedPoint) {
      VLOG(1) << "Havent yet created a projected point";
      math::IntersectionResult intersectionResult;
      double t1, t2;
      if (math::equal(leftInterval.first, leftInterval.second)) {
        VLOG(1) << absl::StreamFormat("Left interval has length %.20f", math::distance(leftInterval.first, leftInterval.second));
        // Left interval has 0-length, create a fake temporary vector for an intersection test.
        Vector tangentStart, tangentEnd;
        if (!currentInterval.intervalLeftIsConstraintVertex()) {
          // How do we create a temporary left interval if the left isnt a constraint?
          // Is there a root point and is this left interval on it?
          if (currentInterval.rootIndex && math::equal(math::distanceSquared(currentInterval.rootPoint, leftInterval.first), agentRadius_*agentRadius_)) {
            std::tie(tangentStart, tangentEnd) = math::createVectorTangentToPointOnCircle(currentInterval.rootPoint, agentRadius_, leftInterval.first);
          } else {
            VLOG(1) << "currentInterval.rootIndex: " << (bool)currentInterval.rootIndex;
            VLOG(1) << absl::StreamFormat("Distance squared from %.5f,%.5f to %.5f,%.5f is %.10f which==?:%v, diff=%.20f", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), leftInterval.first.x(), leftInterval.first.y(), math::distanceSquared(currentInterval.rootPoint, leftInterval.first), math::distanceSquared(currentInterval.rootPoint, leftInterval.first) == agentRadius_*agentRadius_, math::distanceSquared(currentInterval.rootPoint, leftInterval.first) - agentRadius_*agentRadius_);
            throw std::runtime_error("Unseen case 0");
          }
        } else {
          std::tie(tangentStart, tangentEnd) = math::createVectorTangentToPointOnCircle(currentInterval.leftPoint, agentRadius_, leftInterval.first);
          VLOG(1) << "Created tmp interval as Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ")";
        }
        intersectionResult = math::intersectForIntervals(tangentStart, tangentEnd, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      } else {
        intersectionResult = math::intersectForIntervals(leftInterval.first, leftInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      }

      if (math::equal(t2, 0.0, kEndpointIntersectionPrecision)) {
        // Left point is actually the left of the edge.
        projectedLeftPoint = successorEdgeLeftPoint;
        projectedLeftIndex = successorEdgeLeftIndex;
      } else if (math::equal(t2, 1.0, kEndpointIntersectionPrecision)) {
        // Left point is actually the right of the edge.
        projectedLeftPoint = successorEdgeRightPoint;
        projectedLeftIndex = successorEdgeRightIndex;
      } else {
        projectedLeftPoint = successorEdgeLeftPoint + t2 * (successorEdgeRightPoint - successorEdgeLeftPoint);
        VLOG(1) << "Created projected point " << projectedLeftPoint.x() << ',' << projectedLeftPoint.y();
      }
    }

    if (!currentInterval.intervalLeftIsConstraintVertex()) {
      // Left of interval is not a constraint, but the interval could intersect with a constraint on the way to the successor edge.
      // Extend left interval out to successor.
      VLOG(1) << "Checking if left interval intersects with a constraint on the way to the successor edge";
      double i1, i2;
      auto intersectionResult = math::intersectForIntervals(leftInterval.first, leftInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &i1, &i2);
      if (!math::lessThan(i2, 0.0) && !math::lessThan(1.0, i2)) {
        VLOG(1) << "This does intersect with the successor edge";
        const Vector leftIntervalIntersectionWithSuccessorEdge = successorEdgeLeftPoint + (successorEdgeRightPoint - successorEdgeLeftPoint) * i2;
        // For now, we only check if this intersects with the currrent entry edge's left.
        // TODO: Check if this line intersects with any other constraint vertices within range.
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        if (isAConstraintVertexForState(currentState, currentEdgeLeftIndex)) {
          VLOG(1) << "Left of current edge is a constraint";
          Vector intersectionPoint1, intersectionPoint2;
          auto intersectionResult2 = math::lineSegmentIntersectsWithCircle(leftInterval.first, leftIntervalIntersectionWithSuccessorEdge, currentEdgeLeftPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
          // Weed out the tangential intersections.
          const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, leftIntervalIntersectionWithSuccessorEdge, currentEdgeLeftPoint, agentRadius_, intersectionResult2, intersectionPoint1, intersectionPoint2);
          if (actuallyIntersected) {
            VLOG(1) << "Augmented left interval Segment(" << '(' << leftInterval.first.x() << ',' << leftInterval.first.y() << "),(" << leftIntervalIntersectionWithSuccessorEdge.x() << ',' << leftIntervalIntersectionWithSuccessorEdge.y() << ')' << ") intersects with left of current edge point " << currentEdgeLeftIndex;
            return {currentEdgeLeftPoint, currentEdgeLeftIndex, std::nullopt};
          }
        }
      }
    }
  }

  // The right of the interval might not be a vertex, but the right interval extended out to the next edge might pass through the right vertex of the successor edge.
  std::optional<std::pair<Vector, IndexType>> leftConstraint;
  if (!successorEdgeIntersectsWithLeftOfInterval) {
    if (!currentInterval.leftIsRoot() && isAConstraintVertexForState(currentState, successorEdgeLeftIndex) && math::lineSegmentIntersectsWithCircle(currentInterval.leftInterval()->first, projectedLeftPoint, successorEdgeLeftPoint, agentRadius_) == 2) {
      const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.leftInterval()->first, AngleDirection::kNoDirection, successorEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
      double t1_2, t2_2;
      auto intersectionResult2 = math::intersectForIntervals(lineStart, lineEnd, successorEdgeLeftPoint, successorEdgeRightPoint, &t1_2, &t2_2);
      (void)intersectionResult2;
      projectedLeftPoint = lineStart + t1_2 * (lineEnd - lineStart);
      leftConstraint.emplace(successorEdgeLeftPoint, successorEdgeLeftIndex);
    } else if (currentInterval.intervalLeftIsConstraintVertex()) {
      leftConstraint.emplace(currentInterval.leftPoint, *currentInterval.leftIndex);
    }
  }

  // If the root of the interval is a constraint, the left of the interval is not a constraint, and the left of the interval is inside the root, return the root as a left constraint.
  if (!leftConstraint && currentInterval.rootIndex && currentInterval.rootDirection != AngleDirection::kNoDirection && !currentInterval.intervalLeftIsConstraintVertex()) {
    if (math::distanceSquared(currentInterval.rootPoint, currentInterval.leftPoint) < agentRadius_*agentRadius_) {
      VLOG(1) << "Left is inside the root";
      leftConstraint.emplace(currentInterval.rootPoint, *currentInterval.rootIndex);
    }
  }

  return {projectedLeftPoint, projectedLeftIndex, leftConstraint};
}

template<typename NavmeshType>
std::pair<Vector,Vector> Pathfinder<NavmeshType>::createTempRightIntervalForCrossProduct(const IntervalType &currentInterval, const Vector &point) const {
  if (currentInterval.rootDirection == AngleDirection::kNoDirection) {
    // Order doesnt matter?
    return math::createPerpendicularBisector(currentInterval.rightPoint, currentInterval.rootPoint, /*bisectorLength=*/1);
  } else if (currentInterval.rootDirection == AngleDirection::kClockwise) {
    return math::createPerpendicularBisector(point, currentInterval.rootPoint, /*bisectorLength=*/1);
  } else {
    return math::createPerpendicularBisector(currentInterval.rootPoint, point, /*bisectorLength=*/1);
  }
}

template<typename NavmeshType>
std::pair<Vector,Vector> Pathfinder<NavmeshType>::createTempLeftIntervalForCrossProduct(const IntervalType &currentInterval, const Vector &point) const {
  if (currentInterval.rootDirection == AngleDirection::kNoDirection) {
    // Order doesnt matter?
    return math::createPerpendicularBisector(currentInterval.leftPoint, currentInterval.rootPoint, /*bisectorLength=*/1);
  } else if (currentInterval.rootDirection == AngleDirection::kClockwise) {
    return math::createPerpendicularBisector(point, currentInterval.rootPoint, /*bisectorLength=*/1);
  } else {
    return math::createPerpendicularBisector(currentInterval.rootPoint, point, /*bisectorLength=*/1);
  }
}

template<typename NavmeshType>
std::pair<bool,bool> Pathfinder<NavmeshType>::booberGobblin(
                        const std::optional<IndexType> successorEdgeLeftIndex,
                        const std::optional<IndexType> successorEdgeRightIndex,
                        const IntervalType &currentInterval,
                        const Vector &toDestinationStart,
                        const Vector &toDestinationEnd) const {
  VLOG(1) << absl::StreamFormat("Blop is looking at Segment((%.20f,%.20f),(%.20f,%.20f))", toDestinationStart.x(), toDestinationStart.y(), toDestinationEnd.x(), toDestinationEnd.y());
  if (!currentInterval.leftIsRoot() && !currentInterval.leftInterval()) {
    throw std::runtime_error("left is not root and dont have an interval");
  }
  if (!currentInterval.rightIsRoot() && !currentInterval.rightInterval()) {
    throw std::runtime_error("right is not root and dont have an interval");
  }
  if (!currentInterval.state.hasEntryEdgeIndex()) {
    throw std::runtime_error("This function expects the current interval to have a state with a entry edge. i.e., not the Start state");
  }
  bool finalIsLeftOfLeftInterval;
  bool finalIsRightOfRightInterval;

  // Handle the left of the interval
  if (currentInterval.leftIsRoot()) {
    finalIsLeftOfLeftInterval = false;
  } else {
    if (!currentInterval.leftInterval()) {
      // We must have a left interval.
      throw std::runtime_error("Left is not root and dont have a left interval");
    }
    const LineSegment &leftInterval = *currentInterval.leftInterval();
    if (math::equal(leftInterval.first, leftInterval.second)) {
      // Left interval is 0-length.
      // I think this only occurs when right is root and the left of the currentInterval is another constraint touching the constraint of the right.
      // Create a new temporary interval that would have the same direction/angle as if this interval had an actual length.
      const auto [tmpStart, tmpEnd] = createTempLeftIntervalForCrossProduct(currentInterval, leftInterval.first);
      // Use the temporary interval for angle comparison
      finalIsLeftOfLeftInterval = math::lessThan(0.0, math::crossProductForSign(tmpStart, tmpEnd, toDestinationStart, toDestinationEnd));
    } else {
      // Left interval is not 0-length.
      if (currentInterval.leftIntervalToCurrentEntryEdge()) {
        const LineSegment &leftIntervalToCurrentEntryEdge = *currentInterval.leftIntervalToCurrentEntryEdge();
        if (currentInterval.leftIndex) {
          // leftIntervalToCurrentEntryEdge is useful.
          // Which end is closer?
          const auto lengthOfLeftInterval = math::distanceSquared(leftInterval.first, leftInterval.second);
          const auto lengthOfLeftIntervalToCurrentEntryEdge = math::distanceSquared(leftIntervalToCurrentEntryEdge.first, leftIntervalToCurrentEntryEdge.second);
          const auto lengthOfToDestination = math::distanceSquared(toDestinationStart, toDestinationEnd);
          // If the length of the given segment is longer than the left interval, then use the left interval for the cross product
          if (lengthOfToDestination >= lengthOfLeftInterval) {
            finalIsLeftOfLeftInterval = math::lessThan(0.0, math::crossProductForSign(leftInterval.first, leftInterval.second, toDestinationStart, toDestinationEnd));
          } else {
            // The given segment is shorter than the left interval
            if (lengthOfLeftIntervalToCurrentEntryEdge < lengthOfLeftInterval) {
              // Interval to entry edge is closer, use that as the interval for our cross product.
              finalIsLeftOfLeftInterval = math::lessThan(0.0, math::crossProductForSign(leftIntervalToCurrentEntryEdge.first, leftIntervalToCurrentEntryEdge.second, toDestinationStart, toDestinationEnd));
            } else {
              // Regular interval is closer.
              finalIsLeftOfLeftInterval = math::lessThan(0.0, math::crossProductForSign(leftInterval.first, leftInterval.second, toDestinationStart, toDestinationEnd));
            }
          }
        } else {
          // Dont need leftIntervalToCurrentEntryEdge.
          finalIsLeftOfLeftInterval = math::lessThan(0.0, math::crossProductForSign(leftInterval.first, leftInterval.second, toDestinationStart, toDestinationEnd));
        }
      } else {
        // We only have the left interval.
        throw std::runtime_error("When does this happen?");
        // We will have a leftIntervalToCurrentEntryEdge if left is not root and the current state has an entry edge.
        // We can only get here if the left is not the root, so this will only happen if we have no entry edge.
        // We can only have no entry edge if this function is called for the start state, which it is not.
        // TODO: If this function is called for the start state, this exception is expected to be thrown.
      }
    }
  }

  // Handle the right of the interval
  if (currentInterval.rightIsRoot()) {
    finalIsRightOfRightInterval = false;
  } else {
    if (!currentInterval.rightInterval()) {
      // We must have a right interval.
      throw std::runtime_error("Right is not root and dont have a right interval");
    }
    const LineSegment &rightInterval = *currentInterval.rightInterval();
    if (math::equal(rightInterval.first, rightInterval.second)) {
      // Right interval is 0-length.
      // I think this only occurs when left is root and the right of the currentInterval is another constraint touching the constraint of the left.
      // Create a new temporary interval that would have the same direction/angle as if this interval had an actual length.
      const auto [tmpStart, tmpEnd] = createTempRightIntervalForCrossProduct(currentInterval, rightInterval.first);
      // Use the temporary interval for angle comparison
      finalIsRightOfRightInterval = math::lessThan(0.0, math::crossProductForSign(toDestinationStart, toDestinationEnd, tmpStart, tmpEnd));
    } else {
      // Right interval is not 0-length.
      if (currentInterval.rightIntervalToCurrentEntryEdge()) {
        const LineSegment &rightIntervalToCurrentEntryEdge = *currentInterval.rightIntervalToCurrentEntryEdge();
        if (currentInterval.rightIndex) {
          // rightIntervalToCurrentEntryEdge is useful.
          // Which end is closer?
          const auto lengthOfRightInterval = math::distanceSquared(rightInterval.first, rightInterval.second);
          const auto lengthOfRightIntervalToCurrentEntryEdge = math::distanceSquared(rightIntervalToCurrentEntryEdge.first, rightIntervalToCurrentEntryEdge.second);
          const auto lengthOfToDestination = math::distanceSquared(toDestinationStart, toDestinationEnd);
          // If the length of the given segment is longer than the right interval, then use the right interval for the cross product
          if (lengthOfToDestination >= lengthOfRightInterval) {
            finalIsRightOfRightInterval = math::lessThan(0.0, math::crossProductForSign(toDestinationStart, toDestinationEnd, rightInterval.first, rightInterval.second));
          } else {
            // The given segment is shorter than the right interval
            if (lengthOfRightIntervalToCurrentEntryEdge < lengthOfRightInterval) {
              // Interval to entry edge is closer, use that as the interval for our cross product.
              finalIsRightOfRightInterval = math::lessThan(0.0, math::crossProductForSign(toDestinationStart, toDestinationEnd, rightIntervalToCurrentEntryEdge.first, rightIntervalToCurrentEntryEdge.second));
              // throw std::runtime_error("Looking for this new case");
            } else {
              // Regular interval is closer.
              finalIsRightOfRightInterval = math::lessThan(0.0, math::crossProductForSign(toDestinationStart, toDestinationEnd, rightInterval.first, rightInterval.second));
            }
          }
        } else {
          // Dont need rightIntervalToCurrentEntryEdge.
          finalIsRightOfRightInterval = math::lessThan(0.0, math::crossProductForSign(toDestinationStart, toDestinationEnd, rightInterval.first, rightInterval.second));
        }
      } else {
        // We only have the right interval.
        throw std::runtime_error("When does this happen?");
        // We will have a rightIntervalToCurrentEntryEdge if right is not root and the current state has an entry edge.
        // We can only get here if the right is not the root, so this will only happen if we have no entry edge.
        // We can only have no entry edge if this function is called for the start state, which it is not.
        // TODO: If this function is called for the start state, this exception is expected to be thrown.
      }
    }
  }
  return {finalIsLeftOfLeftInterval, finalIsRightOfRightInterval};
}

template<typename NavmeshType>
double Pathfinder<NavmeshType>::costFromIntervalToGoal(const IntervalType &interval, const Vector &goalPoint) const {
  if (interval.isGoal) {
    // We know that the root has direct line of sight to the goal.
    return interval.costToRoot + math::distance(interval.rootPoint, goalPoint); // TODO(cost): Add (estimate of) distance traveled around the interval.root, if any.
  }
  if (interval.rootIndex) {
    if (interval.leftIndex && *interval.rootIndex == *interval.leftIndex &&
        interval.rightIndex && *interval.rootIndex == *interval.rightIndex) {
      // Root, left, and right are all the same.
      if (agentRadius_ > 0.0) {
        throw std::runtime_error("Interval is weird");
      }
    }
    // If the root is one of the ends of the interval; simply find the distance to the goal.
    if (interval.leftIndex && *interval.rootIndex == *interval.leftIndex) {
      if (interval.intervalLeftIsConstraintVertex()) {
        const auto pointAwayFromConstraint = math::extendLineSegmentToLength(interval.leftPoint, interval.rightPoint, agentRadius_);
        return interval.costToRoot + math::distance(pointAwayFromConstraint, goalPoint);
      } else {
        return interval.costToRoot + math::distance(interval.leftPoint, goalPoint);
      }
    } else if (interval.rightIndex && *interval.rootIndex == *interval.rightIndex) {
      if (interval.intervalRightIsConstraintVertex()) {
        const auto pointAwayFromConstraint = math::extendLineSegmentToLength(interval.rightPoint, interval.leftPoint, agentRadius_);
        return interval.costToRoot + math::distance(pointAwayFromConstraint, goalPoint);
      } else {
        return interval.costToRoot + math::distance(interval.rightPoint, goalPoint);
      }
    }
  }
  // First check if the goal and root are on the same side of the interval.
  Vector goal = goalPoint;
  if (!math::equal(interval.rightPoint, interval.leftPoint)) {
    // If left and right of interval are the same point, there's no reflecting to do.
    const auto leftToRightVector = interval.rightPoint - interval.leftPoint;
    const auto leftToRootVector = interval.rootPoint - interval.leftPoint;
    auto leftToGoalVector = goal - interval.leftPoint;
    const bool a = (math::crossProductForSign(leftToRootVector, leftToRightVector) > 0);
    const bool b = (math::crossProductForSign(leftToGoalVector, leftToRightVector) > 0);
    if (a == b) {
      // Both the root and the goal are on the same side of the edge. Reflect the goal over the edge.
      const auto tmpOldGoal = goal;
      goal = math::reflectPointOverLine(goal, interval.leftPoint, interval.rightPoint);
      leftToGoalVector = goal - interval.leftPoint;
    }
  }

  Vector leftPointAwayFromConstraints, rightPointAwayFromConstraints;
  if (interval.intervalLeftIsConstraintVertex()) {
    leftPointAwayFromConstraints = math::extendLineSegmentToLength(interval.leftPoint, interval.rightPoint, agentRadius_);
  } else {
    // TODO: It could be the case that some other constraint will push us away from this point.
    leftPointAwayFromConstraints = interval.leftPoint;
  }
  if (interval.intervalRightIsConstraintVertex()) {
    rightPointAwayFromConstraints = math::extendLineSegmentToLength(interval.rightPoint, interval.leftPoint, agentRadius_);
  } else {
    // TODO: It could be the case that some other constraint will push us away from this point.
    rightPointAwayFromConstraints = interval.rightPoint;
  }
  double interval1, interval2;
  const auto intersectionResult = math::intersectForIntervals(interval.rootPoint, goal, leftPointAwayFromConstraints, rightPointAwayFromConstraints, &interval1, &interval2);
  if (intersectionResult == math::IntersectionResult::kOne ||
      intersectionResult == math::IntersectionResult::kInfinite) {
    // The lines directly overlap, simply use the distance between root and goal.
    return interval.costToRoot + math::distance(interval.rootPoint, goal); // TODO(cost): Add (estimate of) distance traveled around the interval.root, if any.
  } else {
    // No line segment intersection.
    if (interval2 < 0.0) {
      // Lines intersect to the left.
      return interval.costToRoot + math::distance(interval.rootPoint, leftPointAwayFromConstraints) + math::distance(leftPointAwayFromConstraints, goal);
    } else {
      // Lines intersect to the right.
      return interval.costToRoot + math::distance(interval.rootPoint, rightPointAwayFromConstraints) + math::distance(rightPointAwayFromConstraints, goal);
    }
  }
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::pushSuccessor(const IntervalType *currentInterval,
                                            IntervalType successorInterval,
                                            IntervalHeapType &intervalHeap,
                                            IntervalSetType &visited,
                                            IntervalSetType &pushed,
                                            PreviousIntervalMapType &previous,
                                            const Vector &goalPoint,
                                            typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const {
  VLOG(1) << "Want to push " << successorInterval.toString();
  if (successorInterval.rootIndex &&
      math::distanceSquared(successorInterval.leftPoint, successorInterval.rootPoint) <= agentRadius_*agentRadius_ &&
      math::distanceSquared(successorInterval.rightPoint, successorInterval.rootPoint) <= agentRadius_*agentRadius_) {
    // Both the left and right of the interval are inside the root, which is a constraint.
    VLOG(1) << (bool)successorInterval.leftIndex << ',' << (bool)successorInterval.rightIndex;
    VLOG(1) << "Discarding invalid interval";
    return;
  }
  // TODO: It seems useful to check if the interval left and right overlap, but nothing is failing if we dont. Why is that?
  if (successorInterval.rootIndex) {
    // What if the left of the interval is not a constraint and is inside the root?
    bool leftIsConstraint = successorInterval.leftIndex && isAConstraintVertexForState(successorInterval.state, *successorInterval.leftIndex);
    if (!leftIsConstraint) {
      if (math::distanceSquared(successorInterval.rootPoint, successorInterval.leftPoint) <= agentRadius_*agentRadius_) {
        // Update left to be root.
        successorInterval.leftPoint = successorInterval.rootPoint;
        successorInterval.leftIndex = successorInterval.rootIndex;
      }
    }
    // What if the right of the interval is not a constraint and is inside the root?
    bool rightIsConstraint = successorInterval.rightIndex && isAConstraintVertexForState(successorInterval.state, *successorInterval.rightIndex);
    if (!rightIsConstraint) {
      if (math::distanceSquared(successorInterval.rootPoint, successorInterval.rightPoint) <= agentRadius_*agentRadius_) {
        // Update right to be root.
        successorInterval.rightPoint = successorInterval.rootPoint;
        successorInterval.rightIndex = successorInterval.rootIndex;
      }
    }
  }
  if (visited.find(successorInterval) != visited.end()) {
    // Already visited this; don't push it again.
    VLOG(1) << "Already visited!";
    return;
  }
  if (pushed.find(successorInterval) != pushed.end()) {
    // Already pushed this; don't push it again.
    VLOG(1) << "Already pushed!";
    return;
  }

  successorInterval.setIntervalLeftIsConstraintVertex(successorInterval.leftIndex && isAConstraintVertexForState(successorInterval.state, *successorInterval.leftIndex));
  successorInterval.setIntervalRightIsConstraintVertex(successorInterval.rightIndex && isAConstraintVertexForState(successorInterval.state, *successorInterval.rightIndex));
  intervalHeap.emplace(successorInterval, costFromIntervalToGoal(successorInterval, goalPoint));
  pushed.emplace(successorInterval);
  auto it = previous.find(successorInterval);
  if (it != previous.end()) {
    VLOG(1) << "We already have a previous for this state! Weird";
    return;
  }
  if (currentInterval != nullptr) {
    previous.emplace(successorInterval, *currentInterval);
  }

  if constexpr (kProduceDebugAnimationData_) {
    debugAStarInfo.intervals.emplace_back(successorInterval, PathfindingAStarInfo::PushOrPop::kPush);
  }
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::handleStartStateSuccessor(const State &successorState,
                                                        const Vector &startPoint,
                                                        const State &startState,
                                                        const Vector &goalPoint,
                                                        IntervalHeapType &intervalHeap,
                                                        IntervalSetType &visited,
                                                        IntervalSetType &pushed,
                                                        PreviousIntervalMapType &previous,
                                                        typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const {
  if (VLOG_IS_ON(1)) {
    std::cerr << std::endl;
  }
  VLOG(1) << "Working with starting state successor: " << successorState.toString();
  const auto [leftPoint, leftIndex, rightPoint, rightIndex] = getLeftAndRight(successorState);
  // Create the interval for this successor state.
  // It is not always possible to go directly to the successor (when we have a radius).
  bool leftIsConstraint = isAConstraintVertexForState(startState, leftIndex);
  bool rightIsConstraint = isAConstraintVertexForState(startState, rightIndex);
  // Shorten the successor edge to within the constraints at the left and right.
  Vector leftOfEdge = leftPoint;
  if (leftIsConstraint) {
    leftOfEdge = math::extendLineSegmentToLength(leftPoint, rightPoint, agentRadius_);
  }
  Vector rightOfEdge = rightPoint;
  if (rightIsConstraint) {
    rightOfEdge = math::extendLineSegmentToLength(rightPoint, leftPoint, agentRadius_);
  }
  VLOG(1) << absl::StreamFormat("Adjusted edge: Segment((%.20f,%.20f),(%.20f,%.20f))", leftOfEdge.x(), leftOfEdge.y(), rightOfEdge.x(), rightOfEdge.y());
  bool turnedAroundAConstraint{false};
  if (leftIsConstraint) {
    std::optional<bool> optionalNeedToTurnAroundLeft;
    // If the adjusted edge is 0-length, we only turn around this constraint if right is not a constraint or left is closer.
    if (math::equal(leftOfEdge, rightOfEdge)) { // TODO: Should probably be an exact comparison
      VLOG(1) << "Adjusted edge has 0-length";
      if (!rightIsConstraint || math::lessThan(math::distanceSquared(startPoint, leftPoint), math::distanceSquared(startPoint, rightPoint))) {
        // Turn around left
        VLOG(1) << "Right is not a constraint or left is closer, turn around it";
        optionalNeedToTurnAroundLeft = true;
      } else {
        // Don't turn around left.
        VLOG(1) << "Right gets in our way, dont turn around left";
        optionalNeedToTurnAroundLeft = false;
      }
    } else {
      // Create a tangent line from the start point to the left point.
      const auto [startToLeftCircleStart, startToLeftCircleEnd] = math::createCircleConsciousLine(startPoint, AngleDirection::kNoDirection, leftPoint, AngleDirection::kCounterclockwise, agentRadius_);
      // Check if this tangent line eventually intersects with the adjusted edge.
      double i1, i2;
      const auto intersectionResult = math::intersectForIntervals(startToLeftCircleStart, startToLeftCircleEnd, leftOfEdge, rightOfEdge, &i1, &i2);
      if (math::greaterThanOrEqual(i1, 0.0) &&
          math::greaterThanOrEqual(i2, 0.0) &&
          math::lessThanOrEqual(i2, 1.0)) {
        // The tangent line intersects with the adjusted edge; no need to turn
        VLOG(1) << absl::StreamFormat("The tangent line Segment((%.14f,%.14f),(%.14f,%.14f)) intersects with the adjusted edge; no need to turn", startToLeftCircleStart.x(), startToLeftCircleStart.y(), startToLeftCircleEnd.x(), startToLeftCircleEnd.y());
        optionalNeedToTurnAroundLeft = false;
      } else {
        // Check if the tangent intersects with the right constraint (if it is a constraint) before hitting the left constraint.
        VLOG(1) << "Created tangent line (Segment(" << startToLeftCircleStart << ',' << startToLeftCircleEnd << ")) intersects with the adjusted edge; checking if we hit the right vertex first.";
        if (rightIsConstraint) {
          const auto circleIntersectionCount = math::lineSegmentIntersectsWithCircle(startToLeftCircleStart, startToLeftCircleEnd, rightPoint, agentRadius_);
          VLOG(1) << "Circle intersection count: " << circleIntersectionCount;
          if (circleIntersectionCount < 2) {
            // We did not hit the right vertex first; turn around the left vertex.
            if (circleIntersectionCount == 1) {
              // This can happen if we start inside the right constraint
              if (math::greaterThanOrEqual(math::distanceSquared(startPoint, rightPoint), agentRadius_*agentRadius_)) {
                throw std::runtime_error("Circle intersection count == 1; not inside right constraint; curious to see this case");
              } else {
                // Inside right constraint.
                // TODO: We'll crash a little later, because this is an invalid starting positon. Should we just start by checking if we're in an invalid starting positon at the beginning of the algorithm?
              }
            }
            optionalNeedToTurnAroundLeft = true;
          } else {
            // We hit the right vertex before getting to the left vertex.
            VLOG(1) << "Dont turn around the left; we hit the right";
            optionalNeedToTurnAroundLeft = false;
          }
        } else {
          optionalNeedToTurnAroundLeft = true;
        }
      }
    }
    if (!optionalNeedToTurnAroundLeft) {
      throw std::runtime_error("Missed a case");
    }
    if (*optionalNeedToTurnAroundLeft) {
      IntervalType interval(successorState);
      interval.setRoot(leftPoint, leftIndex, AngleDirection::kCounterclockwise);
      interval.setLeft(leftPoint, leftIndex);
      interval.setRight(rightPoint, rightIndex);
      // TODO: set cost.
      VLOG(1) << "Pushing initial successor " << interval.toString();
      pushSuccessor(/*currentInterval=*/nullptr, interval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      turnedAroundAConstraint = true;
    }
  }
  if (rightIsConstraint) {
    std::optional<bool> optionalNeedToTurnAroundRight;
    // If the adjusted edge is 0-length, we only turn around this constraint if left is not a constraint or right is closer.
    if (math::equal(leftOfEdge, rightOfEdge)) {
      VLOG(1) << "Adjusted edge has 0-length";
      if (!leftIsConstraint || math::lessThan(math::distanceSquared(startPoint, rightPoint), math::distanceSquared(startPoint, leftPoint))) {
        // Turn around right
        VLOG(1) << "Left is not a constraint or right is closer, turn around it";
        optionalNeedToTurnAroundRight = true;
      } else {
        // Don't turn around right.
        VLOG(1) << "Left gets in our way, dont turn around right";
        optionalNeedToTurnAroundRight = false;
      }
    } else {
      // Create a tangent line from the start point to the right point.
      const auto [startToRightCircleStart, startToRightCircleEnd] = math::createCircleConsciousLine(startPoint, AngleDirection::kNoDirection, rightPoint, AngleDirection::kClockwise, agentRadius_);
      // Check if this tangent line eventually intersects with the adjusted edge.
      double i1, i2;
      const auto intersectionResult = math::intersectForIntervals(startToRightCircleStart, startToRightCircleEnd, leftOfEdge, rightOfEdge, &i1, &i2);
      if (math::greaterThanOrEqual(i1, 0.0) &&
          math::greaterThanOrEqual(i2, 0.0) &&
          math::lessThanOrEqual(i2, 1.0)) {
        // The tangent line intersects with the adjusted edge; no need to turn
        VLOG(1) << absl::StreamFormat("The tangent line Segment((%.14f,%.14f),(%.14f,%.14f)) intersects with the adjusted edge; no need to turn", startToRightCircleStart.x(), startToRightCircleStart.y(), startToRightCircleEnd.x(), startToRightCircleEnd.y());
        // TODO: This should really be using bendThatMfRoundTheRight.
        optionalNeedToTurnAroundRight = false;
      } else {
        // Check if the tangent intersects with the left constraint (if it is a constraint) before hitting the right constraint.
        VLOG(1) << "Created tangent line intersects with the edge; checking if we hit the left vertex first.";
        if (leftIsConstraint) {
          const auto circleIntersectionCount = math::lineSegmentIntersectsWithCircle(startToRightCircleStart, startToRightCircleEnd, leftPoint, agentRadius_);
          VLOG(1) << "Circle intersection count: " << circleIntersectionCount;
          if (circleIntersectionCount < 2) {
            // We did not hit the left vertex first; turn around the right vertex.
            if (circleIntersectionCount == 1) {
              if (math::greaterThanOrEqual(math::distanceSquared(startPoint, leftPoint), agentRadius_*agentRadius_)) {
                throw std::runtime_error("Circle intersection count == 1; not inside left constraint; curious to see this case");
              } else {
                // Inside left constraint.
                throw std::runtime_error("Circle intersection count == 1; Inside left constraint; curious to see this case");
                // This is different from the rhs case above. The rhs case assumes that we'll crash later and we do because of this case. However, if we start inside the right constraint, we wont even get here.
              }
            }
            optionalNeedToTurnAroundRight = true;
          } else {
            // We hit the left vertex before getting to the right vertex.
            VLOG(1) << "Dont turn around the right; we hit the left";
            optionalNeedToTurnAroundRight = false;
          }
        } else {
          optionalNeedToTurnAroundRight = true;
        }
      }
    }
    if (!optionalNeedToTurnAroundRight) {
      throw std::runtime_error("Missed a case");
    }
    if (*optionalNeedToTurnAroundRight) {
      if (turnedAroundAConstraint) {
        // VLOG(1) << "Wait, we already turned around a constraint";
        throw std::runtime_error("Wait, we already turned around a constraint");
      }
      IntervalType interval(successorState);
      interval.setRoot(rightPoint, rightIndex, AngleDirection::kClockwise);
      interval.setLeft(leftPoint, leftIndex);
      interval.setRight(rightPoint, rightIndex);
      // TODO: set cost.
      VLOG(1) << "Pushing initial successor " << interval.toString();
      pushSuccessor(/*currentInterval=*/nullptr, interval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      turnedAroundAConstraint = true;
    }
  }
  // Did not need to turn around a constraint; we can directly reach the edge.
  if (!turnedAroundAConstraint) {
    IntervalType interval(successorState);
    interval.setRoot(startPoint);
    interval.setLeft(leftPoint, leftIndex);
    interval.setRight(rightPoint, rightIndex);
    VLOG(1) << "[2] Pushing initial successor " << interval.toString();
    pushSuccessor(/*currentInterval=*/nullptr, interval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
  }
  if (VLOG_IS_ON(1)) {
    std::cout << std::endl;
  }
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::expandInterval(IntervalType &currentInterval,
                                             const Vector &startPoint,
                                             const Vector &goalPoint,
                                             const State &goalState,
                                             IntervalHeapType &intervalHeap,
                                             IntervalSetType &visited,
                                             IntervalSetType &pushed,
                                             PreviousIntervalMapType &previous,
                                             typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const {
  const State &currentState = currentInterval.state;
  checkLeftAndRightConstraints(currentInterval);

  // Now, expand this interval and push all successors.
  const auto successorStates = navmesh_.getSuccessors(currentState, goalState, agentRadius_); // TODO: I'm not sure I like the concept of the navmesh checking if we fit through the triangle.
  if (successorStates.empty()) {
    VLOG(1) << "No successor states. Nothing to do.";
  }
  for (const State &successorState : successorStates) {
    if (successorIsInsideIntervalConstraint(currentInterval, successorState)) {
      VLOG(1) << "Successor is inside constraint of interval, skipping " << successorState.toString();
      continue;
    }
    if (!canFitThroughEdge(currentInterval.state, successorState.getEntryEdgeIndex())) {
      continue;
    }
    // Create the line segments for the sides of the interval. These are based on the radii of the constraints.
    buildLeftIntervals(currentInterval);
    buildRightIntervals(currentInterval);
    if (successorState.isGoal()) {
      // Handle the goal state separately from the other states, since it's a single point and not a triangle and thus requires a bit of special handling.
      handleGoalSuccessor(currentInterval, successorState, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
    } else {
      handleNormalSuccessor(currentState, successorState, currentInterval, goalPoint, intervalHeap, visited, pushed, previous, debugAStarInfo);
    }
  }
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::buildResultFromGoalInterval(const IntervalType &goalInterval,
                                                          const Vector &startPoint,
                                                          const Vector &goalPoint,
                                                          PreviousIntervalMapType &previous,
                                                          PathfindingResult &result) const {
  if (!goalInterval.rootIndex) {
    // Must be a straight path from the start to the goal
    result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(startPoint, goalPoint)));
    return;
  }

  struct Root {
    Root(IndexType index, AngleDirection dir) : rootIndex(index), direction(dir) {}
    IndexType rootIndex;
    AngleDirection direction;
  };
  std::vector<Root> rootsOnPath;
  rootsOnPath.emplace_back(*goalInterval.rootIndex, goalInterval.rootDirection);
  VLOG(1) << "  " << goalInterval.toString();
  auto it = previous.find(goalInterval);
  while (it != previous.end()) {
    VLOG(1) << "  " << it->second.toString();
    if (it->second.rootIndex && *it->second.rootIndex != rootsOnPath.back().rootIndex) {
      // New previous root index found.
      VLOG(1) << "New previous root found";
      rootsOnPath.emplace_back(*it->second.rootIndex, it->second.rootDirection);
    }
    it = previous.find(it->second);
  }
  std::reverse(rootsOnPath.begin(), rootsOnPath.end());
  {
    std::stringstream ss;
    ss << "Roots are [ ";
    for (const auto &i : rootsOnPath) {
      ss << i.rootIndex;
      if (i.direction == AngleDirection::kNoDirection) {
        ss << "NoDirection";
      } else if (i.direction == AngleDirection::kClockwise) {
        ss << "Clockwise";
      } else {
        ss << "Counterclockwise";
      }
      ss << ", ";
    }
    ss << ']';
    VLOG(1) << ss.str();
  }
  if (!rootsOnPath.empty()) {
    std::vector<std::pair<Vector, AngleDirection>> verticesOnPath;
    verticesOnPath.reserve(rootsOnPath.size()+2);
    // Add the start point.
    verticesOnPath.emplace_back(startPoint, AngleDirection::kNoDirection);
    // Add all of the vertices.
    for (const auto &i : rootsOnPath) {
      verticesOnPath.emplace_back(navmesh_.getVertex(i.rootIndex), i.direction);
    }
    // Add the goal point.
    verticesOnPath.emplace_back(goalPoint, AngleDirection::kNoDirection);
    VLOG(1) << rootsOnPath.size() << " root(s) on the path";
    VLOG(1) << verticesOnPath.size() << " vertices(s) on the path";
    {
      // Get the points for the line between the two vertices.
      const auto [lineStart, lineEnd] = math::createCircleConsciousLine(verticesOnPath.at(0).first, verticesOnPath.at(0).second, verticesOnPath.at(1).first, verticesOnPath.at(1).second, agentRadius_);
      // Create a straight segment from the start to the second point.
      result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(lineStart, lineEnd)));
      // Start an arc at the end of this straight segment.
      result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(verticesOnPath.at(1).first, agentRadius_, verticesOnPath.at(1).second)));
      ArcPathSegment *arc = reinterpret_cast<ArcPathSegment*>(result.shortestPath.back().get());
      arc->startAngle = math::angle(verticesOnPath.at(1).first, lineEnd);
      VLOG(1) << "Arc " << (verticesOnPath.at(1).second == AngleDirection::kClockwise ? "cw" : "ccw") << " start at " << verticesOnPath.at(1).first.x() << ',' << verticesOnPath.at(1).first.y() << " has angle " << arc->startAngle << " (to point " << lineEnd.x() << "," << lineEnd.y() << ")";
    }
    for (int i=1; i<verticesOnPath.size()-1; ++i) {
      // Get the points for the line between the two vertices.
      const auto [lineStart, lineEnd] = math::createCircleConsciousLine(verticesOnPath.at(i).first, verticesOnPath.at(i).second, verticesOnPath.at(i+1).first, verticesOnPath.at(i+1).second, agentRadius_);
      // Finish the previous arc.
      ArcPathSegment *previousArc = reinterpret_cast<ArcPathSegment*>(result.shortestPath.back().get());
      if (previousArc == nullptr) {
        throw std::runtime_error("Last point of path is not an arc");
      }
      previousArc->endAngle = math::angle(verticesOnPath.at(i).first, lineStart);
      VLOG(1) << "Arc end at " << verticesOnPath.at(i).first.x() << ',' << verticesOnPath.at(i).first.y() << " has angle " << previousArc->endAngle << " (to point " << lineStart.x() << "," << lineStart.y() << ")";
      if (!math::equal(lineStart, lineEnd)) {
        // Create a straight segment from the start to the second point.
        result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(lineStart, lineEnd)));
      }
      if (i < verticesOnPath.size()-2) {
        // More segments are coming.
        // Start an arc at the end of this straight segment.
        result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(verticesOnPath.at(i+1).first, agentRadius_, verticesOnPath.at(i+1).second)));
        ArcPathSegment *newArc = reinterpret_cast<ArcPathSegment*>(result.shortestPath.back().get());
        newArc->startAngle = math::angle(verticesOnPath.at(i+1).first, lineEnd);
        VLOG(1) << "Arc " << (verticesOnPath.at(i+1).second == AngleDirection::kClockwise ? "cw" : "ccw") << " start at " << verticesOnPath.at(i+1).first.x() << ',' << verticesOnPath.at(i+1).first.y() << " has angle " << newArc->startAngle << " (to point " << lineEnd.x() << "," << lineEnd.y() << ")";
      }
    }
  } else {
    result.shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(startPoint, goalPoint)));
  }
  if (VLOG_IS_ON(1)) {
    for (const auto &i : result.shortestPath) {
      const auto *p = dynamic_cast<const StraightPathSegment*>(i.get());
      if (p != nullptr) {
        VLOG(1) << absl::StreamFormat("Path Segment((%.20f,%.20f),(%.20f,%.20f))", p->startPoint.x(), p->startPoint.y(), p->endPoint.x(), p->endPoint.y());
      }
    }
  }
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::checkLeftAndRightConstraints(IntervalType &currentInterval) const {
  const State &currentState = currentInterval.state;
  if (currentInterval.leftIsRoot() && currentInterval.leftIndex && currentInterval.rootIndex && currentInterval.leftIndex != currentInterval.rootIndex) {
    // These are only possible when there's a shared vertex.
    throw std::runtime_error("Left is root, but left does not have the same index as the root");
  }
  if (currentInterval.rightIsRoot() && currentInterval.rightIndex && currentInterval.rootIndex && currentInterval.rightIndex != currentInterval.rootIndex) {
    // These are only possible when there's a shared vertex.
    throw std::runtime_error("Right is root, but right does not have the same index as the root");
  }
}

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::successorIsInsideIntervalConstraint(const IntervalType &currentInterval, const State &successorState) const {
  // If the successor state is a triangle that fits completely within the radius of one of the sides of our interval, and that side is a constraint, don't even consider this successor because it is impossible to visit.
  const auto triangleIndex = successorState.getTriangleIndex();
  const auto [v1, v2, v3] = navmesh_.getTriangleVertices(triangleIndex);
  const double radiusSquared = agentRadius_*agentRadius_;
  if (currentInterval.intervalLeftIsConstraintVertex()) {
    if (math::distanceSquared(currentInterval.leftPoint, v1) <= radiusSquared &&
        math::distanceSquared(currentInterval.leftPoint, v2) <= radiusSquared &&
        math::distanceSquared(currentInterval.leftPoint, v3) <= radiusSquared) {
      VLOG(1) << "Entire triangle is unreachable because it is completely inside the left of the interval, which is a constraint";
      // Nothing else to do for this successor.
      return true;
    }
  }
  if (currentInterval.intervalRightIsConstraintVertex()) {
    if (math::distanceSquared(currentInterval.rightPoint, v1) <= radiusSquared &&
        math::distanceSquared(currentInterval.rightPoint, v2) <= radiusSquared &&
        math::distanceSquared(currentInterval.rightPoint, v3) <= radiusSquared) {
      VLOG(1) << "Entire triangle is unreachable because it is completely inside the right of the interval, which is a constraint";
      // Nothing else to do for this successor.
      return true;
    }
  }
  if (currentInterval.rootIndex) {
    if (math::distanceSquared(currentInterval.rootPoint, v1) <= radiusSquared &&
        math::distanceSquared(currentInterval.rootPoint, v2) <= radiusSquared &&
        math::distanceSquared(currentInterval.rootPoint, v3) <= radiusSquared) {
      VLOG(1) << "Entire triangle is unreachable because it is completely inside the root of the interval, which is a constraint";
      // Nothing else to do for this successor.
      return true;
    }
  }
  return false;
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::handleNormalSuccessor(const State &currentState,
                             const State &successorState,
                             const IntervalType &currentInterval,
                             const Vector &goalPoint,
                             IntervalHeapType &intervalHeap,
                             IntervalSetType &visited,
                             IntervalSetType &pushed,
                             PreviousIntervalMapType &previous,
                             typename PathfindingResult::DebugAStarInfoType &debugAStarInfo
                             ) const {
  VLOG(1) << "Considering successor state " << successorState.toString();
  const auto [successorEdgeLeftPoint, successorEdgeLeftIndex, successorEdgeRightPoint, successorEdgeRightIndex] = getLeftAndRight(successorState);
  const bool successorLeftIsConstraintVertex = isAConstraintVertexForState(currentState, successorEdgeLeftIndex);
  const bool successorRightIsConstraintVertex = isAConstraintVertexForState(currentState, successorEdgeRightIndex);

  // Do a quick check to see if we can fit through this edge.
  // TODO: This could be done way earlier.
  //  ACTUALLY, getSuccessors can handle this. Currently we pass 0.0 as agent radius, instead we should pass agentRadius_
  if (successorLeftIsConstraintVertex && successorRightIsConstraintVertex) {
    if (math::distanceSquared(successorEdgeLeftPoint, successorEdgeRightPoint) < (4*agentRadius_*agentRadius_)) {
      // Cannot fit.
      // Nothing else to do for this successor.
      return;
    }
  }

  if (doesRightIntervalIntersectWithLeftOfSuccessorEdge(currentInterval, successorLeftIsConstraintVertex, successorEdgeLeftIndex, successorEdgeLeftPoint, successorEdgeRightPoint)) {
    // Cannot reach successor edge.
    return;
  }
  if (doesLeftIntervalIntersectWithRightOfSuccessorEdge(currentInterval, successorRightIsConstraintVertex, successorEdgeRightIndex, successorEdgeLeftPoint, successorEdgeRightPoint)) {
    // Cannot reach successor edge.
    return;
  }

  VLOG(1) << "  successorEdgeLeftIndex: " << successorEdgeLeftIndex << ", successorEdgeRightIndex: " << successorEdgeRightIndex;
  const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);

  internal::RelativePositionToInterval leftVertexRelativePositionToInterval;
  internal::RelativePositionToInterval rightVertexRelativePositionToInterval;
  try {
    leftVertexRelativePositionToInterval = calculateSuccessorLeftRelativeToInterval(currentInterval, currentEdgeLeftIndex, successorEdgeLeftPoint, successorEdgeLeftIndex, successorEdgeRightPoint, successorEdgeRightIndex, successorLeftIsConstraintVertex);
    if (VLOG_IS_ON(1)) {
      std::cout << std::endl;
    }
    rightVertexRelativePositionToInterval = calculateSuccessorRightRelativeToInterval(currentInterval, currentEdgeRightIndex, successorEdgeLeftPoint, successorEdgeLeftIndex, successorEdgeRightPoint, successorEdgeRightIndex, successorRightIsConstraintVertex);
    if (VLOG_IS_ON(1)) {
      std::cout << std::endl;
    }
  } catch (const std::exception &ex) {
    // TODO: It seems that a failure here means that the interval is invalid. I'm not entirely convinced that's the case.
    VLOG(1) << "Failed to calculate relative positions: \"" << ex.what() << "\"";
    return;
  }

  VLOG(1) << "leftVertexRelativePositionToInterval: " << internal::toString(leftVertexRelativePositionToInterval);
  VLOG(1) << "rightVertexRelativePositionToInterval: " << internal::toString(rightVertexRelativePositionToInterval);
  if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsRightOf &&
      rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsLeftOf) {
    // No hope of getting to this successor.
    // throw std::runtime_error("What even is this case?");
    return;
  }

  bool leftEdgeShareVertex = (currentInterval.leftIndex && *currentInterval.leftIndex == successorEdgeLeftIndex);
  bool rightEdgeShareVertex = (currentInterval.rightIndex && *currentInterval.rightIndex == successorEdgeRightIndex);
  if (leftEdgeShareVertex && rightEdgeShareVertex) {
    // Is the same edge.
    // Create a new interval which is the same at the current one but uses the successor state.
    IntervalType successorInterval(successorState);
    successorInterval.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
    successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
    successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
    successorInterval.costToRoot = currentInterval.costToRoot;
    pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
    // Nothing else to do for this successor.
    return;
  }
  constexpr const double kEndpointIntersectionPrecision{1e-12};

  if (!currentInterval.leftIsRoot() && !currentInterval.leftInterval()) {
    throw std::runtime_error("No leftInterval");
  }
  if (!currentInterval.rightIsRoot() && !currentInterval.rightInterval()) {
    throw std::runtime_error("No rightInterval");
  }

  if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsWithin &&
      rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsWithin) {
    // There is just one successor, it is the entire edge which is reachable from the root.
    VLOG(1) << "== Left is within | right is within ==";
    IntervalType successorInterval(successorState);
    successorInterval.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
    successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
    successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
    successorInterval.costToRoot = currentInterval.costToRoot;
    pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
  } else if (rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsWithin) {
    // Left is not within the interval.
    if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsLeftOf) {
      VLOG(1) << "== Left is left | right is within ==";
      // There is for sure one successor, which is created by (the intersection of the left of the interval with the successor edge, right vertex of the successor edge).
      const auto [optionalProjectedLeftPoint, projectedLeftIndex, leftConstraint] = bendThatMfRoundTheLeft( successorEdgeRightPoint,
                                                                                                            successorEdgeRightIndex,
                                                                                                            currentInterval,
                                                                                                            successorEdgeLeftPoint,
                                                                                                            successorEdgeLeftIndex,
                                                                                                            kEndpointIntersectionPrecision,
                                                                                                            currentState);
      // It could be the case that we run into a constraint of the successor edge while turning around the left. If we do, there wont be any projected point.
      if (optionalProjectedLeftPoint) {
        VLOG(1) << "optionalProjectedLeftPoint";
        // This creates an observable successor.
        IntervalType successorInterval1(successorState);
        successorInterval1.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
        successorInterval1.setLeft(*optionalProjectedLeftPoint, projectedLeftIndex);
        successorInterval1.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        successorInterval1.costToRoot = currentInterval.costToRoot;
        pushSuccessor(&currentInterval, successorInterval1, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }

      if (leftConstraint) {
        VLOG(1) << "leftConstraint";
        // Left of this interval is a vertex, this will create a second, non-observable successor.
        IntervalType successorInterval2(successorState);
        successorInterval2.setRoot(leftConstraint->first, leftConstraint->second, AngleDirection::kCounterclockwise);
        successorInterval2.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        if (optionalProjectedLeftPoint) {
          successorInterval2.setRight(*optionalProjectedLeftPoint, projectedLeftIndex);
        } else {
          successorInterval2.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        }
        // Updated root; update cost to get to root.
        const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, leftConstraint->first, AngleDirection::kCounterclockwise, agentRadius_);
        successorInterval2.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval2, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }
    } else {
      VLOG(1) << "== Left is right | right is within ==";
      // Left is right of the interval.
      if (leftVertexRelativePositionToInterval != internal::RelativePositionToInterval::kIsRightOf) {
        throw std::runtime_error("Expecting leftVertexIsRightOf");
      }
      // If the right interval intersects with the right constraint of the successor edge before actually getting to the successor edge, we will use that as a new root.
      const auto canUseRightOfSuccessorAsRoot = [&](){
        if (!currentInterval.rightInterval()) {
          VLOG(1) << "No right interval";
          return false;
        }
        if (!successorRightIsConstraintVertex) {
          VLOG(1) << "Successor right is not a constraint vertex";
          return false;
        }
        const LineSegment &rightInterval = *currentInterval.rightInterval();
        const Vector extendedRightIntervalEnd = math::extendLineSegmentToLength(rightInterval.first, rightInterval.second, math::distance(rightInterval.first, successorEdgeRightPoint));
        Vector intersectionPoint;
        const int intersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, extendedRightIntervalEnd, successorEdgeRightPoint, agentRadius_, &intersectionPoint);
        if (intersectionCount == 0) {
          VLOG(1) << "No intersection";
          return false;
        }
        math::IntersectionResult intersectionResult = math::intersectForIntervals(rightInterval.first, intersectionPoint, successorEdgeLeftPoint, successorEdgeRightPoint);
        if (intersectionResult == math::IntersectionResult::kInfinite) {
          throw std::runtime_error("Infinite intersection would be weird.");
        }
        if (intersectionResult == math::IntersectionResult::kOne) {
          // Actually intersects with the successor edge before reaching the constraint.
          // TODO: What if the intersection is exactly at `intersectionPoint`?
          return false;
        } else {
          // Does not intersect with the successor edge before intersecting with the constraint.
          // This means that the right of the successor edge should be used as the new root.
          return true;
        }
      };
      if (currentInterval.intervalRightIsConstraintVertex()) {
        // This creates a non-observable successor.
        IntervalType successorInterval(successorState);
        successorInterval.setRoot(currentInterval.rightPoint, currentInterval.rightIndex, AngleDirection::kClockwise);
        successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting to have right interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      } else if (canUseRightOfSuccessorAsRoot()) {
        // This creates a non-observable successor.
        IntervalType successorInterval(successorState);
        successorInterval.setRoot(successorEdgeRightPoint, successorEdgeRightIndex, AngleDirection::kClockwise);
        successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        // Updated root; update cost to get to root.
        const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }
    }
  } else if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsWithin) {
    // Right is not within the interval.
    if (rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsRightOf) {
      VLOG(1) << "== Left is within | right is right ==";
      // There is for sure one successor, which is created by (left vertex of the successor edge, the intersection of the right of the interval with the successor edge).
      const auto [optionalProjectedRightPoint, projectedRightIndex, rightConstraint] = bendThatMfRoundTheRight( successorEdgeRightPoint,
                                                                                                                successorEdgeRightIndex,
                                                                                                                currentInterval,
                                                                                                                successorEdgeLeftPoint,
                                                                                                                successorEdgeLeftIndex,
                                                                                                                kEndpointIntersectionPrecision,
                                                                                                                currentState);
      // It could be the case that we run into a constraint of the successor edge while turning around the right. If we do, there wont be any projected point.
      if (!optionalProjectedRightPoint) {
        VLOG(1) << "No optionalProjectedRightPoint";
      } else {
        VLOG(1) << "optionalProjectedRightPoint: " << absl::StreamFormat("%.20f,%.20f", optionalProjectedRightPoint->x(), optionalProjectedRightPoint->y());
      }
      if (!projectedRightIndex) {
        VLOG(1) << "No projectedRightIndex";
      } else {
        VLOG(1) << "projectedRightIndex: " << *projectedRightIndex;
      }
      if (!rightConstraint) {
        VLOG(1) << "No rightConstraint";
      } else {
        VLOG(1) << "rightConstraint: " << absl::StreamFormat("%.20f,%.20f", rightConstraint->first.x(), rightConstraint->first.y()) << rightConstraint->second;
      }
      if (optionalProjectedRightPoint) {
        // This creates an observable successor.
        IntervalType successorInterval1(successorState);
        successorInterval1.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
        successorInterval1.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        successorInterval1.setRight(*optionalProjectedRightPoint, projectedRightIndex);
        successorInterval1.costToRoot = currentInterval.costToRoot;
        pushSuccessor(&currentInterval, successorInterval1, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }

      if (rightConstraint) {
        // Right of this interval is a vertex, this will create a second, non-observable successor.
        IntervalType successorInterval2(successorState);
        successorInterval2.setRoot(rightConstraint->first, rightConstraint->second, AngleDirection::kClockwise);
        if (optionalProjectedRightPoint) {
          successorInterval2.setLeft(*optionalProjectedRightPoint, projectedRightIndex);
        } else {
          successorInterval2.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        }
        successorInterval2.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        // Updated root; update cost to get to root.
        const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, rightConstraint->first, AngleDirection::kClockwise, agentRadius_);
        successorInterval2.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval2, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }
    } else {
      VLOG(1) << "== Left is within | right is left ==";
      // Right is left of the interval.
      if (rightVertexRelativePositionToInterval != internal::RelativePositionToInterval::kIsLeftOf) {
        throw std::runtime_error("Expecting rightVertexIsLeftOf");
      }
      // If the left interval intersects with the left constraint of the successor edge before actually getting to the successor edge, we will use that as a new root.
      const auto canUseLeftOfSuccessorAsRoot = [&](){
        if (!currentInterval.leftInterval()) {
          VLOG(1) << "No left interval";
          return false;
        }
        if (!successorLeftIsConstraintVertex) {
          VLOG(1) << "Successor left is not a constraint vertex";
          return false;
        }
        const LineSegment &leftInterval = *currentInterval.leftInterval();
        const Vector extendedLeftIntervalEnd = math::extendLineSegmentToLength(leftInterval.first, leftInterval.second, math::distance(leftInterval.first, successorEdgeLeftPoint));
        Vector intersectionPoint;
        const int intersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, extendedLeftIntervalEnd, successorEdgeLeftPoint, agentRadius_, &intersectionPoint);
        if (intersectionCount == 0) {
          VLOG(1) << "No intersection";
          return false;
        }
        math::IntersectionResult intersectionResult = math::intersectForIntervals(leftInterval.first, intersectionPoint, successorEdgeLeftPoint, successorEdgeRightPoint);
        if (intersectionResult == math::IntersectionResult::kInfinite) {
          throw std::runtime_error("Infinite intersection would be weird.");
        }
        if (intersectionResult == math::IntersectionResult::kOne) {
          // Actually intersects with the successor edge before reaching the constraint.
          // TODO: What if the intersection is exactly at `intersectionPoint`?
          return false;
        } else {
          // Does not intersect with the successor edge before intersecting with the constraint.
          // This means that the left of the successor edge should be used as the new root.
          return true;
        }
      };
      if (currentInterval.intervalLeftIsConstraintVertex()) {
        // This creates a non-observable successor.
        IntervalType successorInterval(successorState);
        successorInterval.setRoot(currentInterval.leftPoint, currentInterval.leftIndex, AngleDirection::kCounterclockwise);
        successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
        successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting to have left interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      } else if (canUseLeftOfSuccessorAsRoot()) {
        // This creates a non-observable successor.
        IntervalType successorInterval(successorState);
        successorInterval.setRoot(successorEdgeLeftPoint, successorEdgeLeftIndex, AngleDirection::kCounterclockwise);
        successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
        successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
        // Updated root; update cost to get to root.
        const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      }
    }
  } else {
    // Neither point is within the interval.
    // Both are to the left, both are to the right, or left is left and right is right.
    // const double kCircleIntersectionTolerance{1.55e-5};
    if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsLeftOf &&
        rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsLeftOf) {
      VLOG(1) << "== Left is left | right is left ==";
      // Check if we hit the left of the successor edge before getting to the edge.
      const bool hitBlockingIntersection = [&]{
        if (!currentInterval.rightInterval()) {
          return false;
        }
        if (currentInterval.intervalRightIsConstraintVertex()) {
          // Might be able to turn around
          // TODO: Think about.
          return false;
        }
        if (!successorLeftIsConstraintVertex) {
          return false;
        }
        const double distance = math::distance(currentInterval.rightInterval()->first, successorEdgeLeftPoint);
        const auto extendedRightIntervalEnd = math::extendLineSegmentToLength(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second, distance);
        int intersectionCount = math::lineSegmentIntersectsWithCircle(currentInterval.rightInterval()->first, extendedRightIntervalEnd, successorEdgeLeftPoint, agentRadius_);
        return intersectionCount > 0;
      }();
      if (!hitBlockingIntersection) {
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting to have a left interval");
        }
        const LineSegment &leftInterval = *currentInterval.leftInterval();
        if (currentInterval.intervalLeftIsConstraintVertex()) {
          // Make sure that we dont hit the right of the interval before getting to the left.
          if (currentInterval.intervalRightIsConstraintVertex()) {
            Vector intersectionPoint1, intersectionPoint2;
            VLOG(1) << "Checking intersection of Segment(" << '(' << leftInterval.first.x() << ',' << leftInterval.first.y() << ')' << ',' << '(' << leftInterval.second.x() << ',' << leftInterval.second.y() << ')' << ") against point " << '(' << currentInterval.rightPoint.x() << ',' << currentInterval.rightPoint.y() << ')';
            const int intersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, leftInterval.second, currentInterval.rightPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
            const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, leftInterval.second, currentInterval.rightPoint, agentRadius_, intersectionCount, intersectionPoint1, intersectionPoint2);
            if (actuallyIntersected) {
              VLOG(1) << "Impossible to get to left interval endpoint";
              // Nothing else to do for this successor.
              return;
            } else {
              VLOG(1) << "No problem getting to left interval endpoint";
            }
          } else {
            VLOG(1) << "Interval right is not a constraint vertex, dont need to worry about hitting it.";
          }
          // This creates a non-observable successor, which is the entire edge.
          IntervalType successorInterval(successorState);
          successorInterval.setRoot(currentInterval.leftPoint, currentInterval.leftIndex, AngleDirection::kCounterclockwise);
          successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
          successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
          // Updated root; update cost to get to root.
          if (!currentInterval.leftInterval()) {
            throw std::runtime_error("Expecting to have left interval");
          }
          successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
          pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
        } else {
          VLOG(1) << "Both are left of, but left of interval is not a constraint vertex";
          // Does the left interval intersect with the entry edge's left constraint before the successor edge?
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          bool turnedAroundCurrentEntryEdgeLeft = false;
          if (isAConstraintVertexForState(currentState, currentEdgeLeftIndex)) {
            VLOG(1) << "Current state entry edge left is a constraint to us, we might be turning around it";
            // Extend the left interval to be long enough to intersect with the vertex.
            const auto newEnd = math::extendLineSegmentToLength(leftInterval.first, leftInterval.second, math::distance(leftInterval.first, currentEdgeLeftPoint));
            VLOG(1) << "Extended left interval is " << absl::StreamFormat("Segment((%.20f,%.20f),(%.20f,%.20f))", leftInterval.first.x(), leftInterval.first.y(), newEnd.x(), newEnd.y());
            Vector iPoint1, iPoint2;
            auto vertexIntersectionResult = math::lineSegmentIntersectsWithCircle(leftInterval.first, newEnd, currentEdgeLeftPoint, agentRadius_, &iPoint1, &iPoint2);
            if (vertexIntersectionResult > 0) {
              VLOG(1) << "Creating one non-observable successor.";
              // This creates a non-observable successor, which is the entire edge.
              IntervalType successorInterval(successorState);
              successorInterval.setRoot(currentEdgeLeftPoint, currentEdgeLeftIndex, AngleDirection::kCounterclockwise);
              successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
              successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
              // Updated root; update cost to get to root.
              const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
              turnedAroundCurrentEntryEdgeLeft = true;
            }
          }
          if (!turnedAroundCurrentEntryEdgeLeft && successorLeftIsConstraintVertex) {
            VLOG(1) << "Successor state entry edge left is a constraint to us, we might be turning around it";
            // Extend the left interval to be long enough to intersect with the vertex.
            const auto newEnd = math::extendLineSegmentToLength(leftInterval.first, leftInterval.second, math::distance(leftInterval.first, successorEdgeLeftPoint));
            VLOG(1) << "Extended left interval is " << absl::StreamFormat("Segment((%.20f,%.20f),(%.20f,%.20f))", leftInterval.first.x(), leftInterval.first.y(), newEnd.x(), newEnd.y());
            Vector iPoint1, iPoint2;
            auto vertexIntersectionResult = math::lineSegmentIntersectsWithCircle(leftInterval.first, newEnd, successorEdgeLeftPoint, agentRadius_, &iPoint1, &iPoint2);
            if (vertexIntersectionResult > 0) {
              VLOG(1) << "Creating one non-observable successor.";
              // This creates a non-observable successor, which is the entire edge.
              IntervalType successorInterval(successorState);
              successorInterval.setRoot(successorEdgeLeftPoint, successorEdgeLeftIndex, AngleDirection::kCounterclockwise);
              successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
              successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
              // Updated root; update cost to get to root
              const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
            }
          }
        }
      }
      VLOG(1) << "___ ________________ ___";
    } else if (leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsRightOf &&
               rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsRightOf) {
      VLOG(1) << "== Left is right | right is right ==";
      // TODO: This isnt always correct ========================================================>
      // TODO: Same for mirrored above
      // Check if we hit the left of the successor edge before getting to the edge.
      const bool hitBlockingIntersection = [&]{
        if (!currentInterval.leftInterval()) {
          return false;
        }
        if (currentInterval.intervalLeftIsConstraintVertex()) {
          // Might be able to turn around
          // TODO: Think about.
          return false;
        }
        if (!successorRightIsConstraintVertex) {
          return false;
        }
        const double distance = math::distance(currentInterval.leftInterval()->first, successorEdgeRightPoint);
        const auto extendedLeftIntervalEnd = math::extendLineSegmentToLength(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second, distance);
        int intersectionCount = math::lineSegmentIntersectsWithCircle(currentInterval.leftInterval()->first, extendedLeftIntervalEnd, successorEdgeRightPoint, agentRadius_);
        return intersectionCount > 0;
      }();
      if (!hitBlockingIntersection) { // TODO: Need a test for this case.
      // <================================================================================================================
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting to have a right interval");
        }
        const LineSegment &rightInterval = *currentInterval.rightInterval();
        if (currentInterval.intervalRightIsConstraintVertex()) {
          // Make sure that we dont hit the left of the interval before getting to the right.
          if (currentInterval.intervalLeftIsConstraintVertex()) {
            Vector intersectionPoint1, intersectionPoint2;
            const int intersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, rightInterval.second, currentInterval.leftPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
            const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, rightInterval.second, currentInterval.leftPoint, agentRadius_, intersectionCount, intersectionPoint1, intersectionPoint2);
            if (actuallyIntersected) {
              VLOG(1) << "Impossible to get to right interval endpoint";
              // Nothing else to do for this successor.
              return;
            } else {
              VLOG(1) << "No problem getting to right interval endpoint";
            }
          } else {
            VLOG(1) << "Interval left is not a constraint vertex, dont need to worry about hitting it.";
          }
          // This creates a non-observable successor, which is the entire edge.
          IntervalType successorInterval(successorState);
          successorInterval.setRoot(currentInterval.rightPoint, currentInterval.rightIndex, AngleDirection::kClockwise);
          successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
          successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
          // Updated root; update cost to get to root.
          if (!currentInterval.rightInterval()) {
            throw std::runtime_error("Expecting to have right interval");
          }
          successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
          pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
        } else {
          VLOG(1) << "Both are right of, but right of interval is not a constraint vertex";
          // Does the right interval intersect with the entry edge's right constraint before the successor edge?
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          bool turnedAroundCurrentEntryEdgeRight = false;
          if (isAConstraintVertexForState(currentState, currentEdgeRightIndex)) {
            VLOG(1) << "Current state entry edge right is a constraint to us, we might be turning around it";
            // Extend the right interval to be long enough to intersect with the vertex.
            const auto newEnd = math::extendLineSegmentToLength(rightInterval.first, rightInterval.second, math::distance(rightInterval.first, currentEdgeRightPoint));
            VLOG(1) << "Extended right interval is " << absl::StreamFormat("Segment((%.20f,%.20f),(%.20f,%.20f))", rightInterval.first.x(), rightInterval.first.y(), newEnd.x(), newEnd.y());
            Vector iPoint1, iPoint2;
            auto vertexIntersectionResult = math::lineSegmentIntersectsWithCircle(rightInterval.first, newEnd, currentEdgeRightPoint, agentRadius_, &iPoint1, &iPoint2);
            if (vertexIntersectionResult > 0) {
              VLOG(1) << "Creating one non-observable successor.";
              // This creates a non-observable successor, which is the entire edge.
              IntervalType successorInterval(successorState);
              successorInterval.setRoot(currentEdgeRightPoint, currentEdgeRightIndex, AngleDirection::kClockwise);
              successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
              successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
              // Updated root; update cost to get to root.
              const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
              turnedAroundCurrentEntryEdgeRight = true;
            }
          }
          if (!turnedAroundCurrentEntryEdgeRight && successorRightIsConstraintVertex) {
            VLOG(1) << "Successor state entry edge right is a constraint to us, we might be turning around it";
            // Extend the right interval to be long enough to intersect with the vertex.
            const auto newEnd = math::extendLineSegmentToLength(rightInterval.first, rightInterval.second, math::distance(rightInterval.first, successorEdgeRightPoint));
            VLOG(1) << "Extended right interval is " << absl::StreamFormat("Segment((%.20f,%.20f),(%.20f,%.20f))", rightInterval.first.x(), rightInterval.first.y(), newEnd.x(), newEnd.y());
            Vector iPoint1, iPoint2;
            auto vertexIntersectionResult = math::lineSegmentIntersectsWithCircle(rightInterval.first, newEnd, successorEdgeRightPoint, agentRadius_, &iPoint1, &iPoint2);
            if (vertexIntersectionResult > 0) {
              VLOG(1) << "Creating one non-observable successor.";
              // This creates a non-observable successor, which is the entire edge.
              IntervalType successorInterval(successorState);
              successorInterval.setRoot(successorEdgeRightPoint, successorEdgeRightIndex, AngleDirection::kClockwise);
              successorInterval.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
              successorInterval.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
              // Updated root; update cost to get to root.
              const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
            }
          }
        }
      }
    } else {
      VLOG(1) << "== Left is left | right is right ==";
      if (!(leftVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsLeftOf &&
            rightVertexRelativePositionToInterval == internal::RelativePositionToInterval::kIsRightOf)) {
        throw std::runtime_error("Assumption voided 2");
      }

      if (VLOG_IS_ON(1)) {
        std::cout << std::endl;
      }
      const auto [optionalProjectedLeftPoint, projectedLeftIndex, leftConstraint] = bendThatMfRoundTheLeft( successorEdgeRightPoint,
                                                                                                            successorEdgeRightIndex,
                                                                                                            currentInterval,
                                                                                                            successorEdgeLeftPoint,
                                                                                                            successorEdgeLeftIndex,
                                                                                                            kEndpointIntersectionPrecision,
                                                                                                            currentState);
      if (VLOG_IS_ON(1)) {
        std::cout << std::endl;
      }
      const auto [optionalProjectedRightPoint, projectedRightIndex, rightConstraint] = bendThatMfRoundTheRight( successorEdgeRightPoint,
                                                                                                                successorEdgeRightIndex,
                                                                                                                currentInterval,
                                                                                                                successorEdgeLeftPoint,
                                                                                                                successorEdgeLeftIndex,
                                                                                                                kEndpointIntersectionPrecision,
                                                                                                                currentState);
      if (VLOG_IS_ON(1)) {
        std::cout << std::endl;
      }
      VLOG(1) << "Result of bendThatMfRoundTheLeft:";
      if (optionalProjectedLeftPoint) {
        VLOG(1) << "optionalProjectedLeftPoint: " << optionalProjectedLeftPoint->x() << ',' << optionalProjectedLeftPoint->y();
      } else {
        VLOG(1) << "optionalProjectedLeftPoint: NONE";
      }
      {
        std::stringstream ss;
        ss << "projectedLeftIndex: ";
        if (projectedLeftIndex) {
          ss << *projectedLeftIndex;
        } else {
          ss << "NONE";
        }
        VLOG(1) << ss.str();
      }
      {
        std::stringstream ss;
        ss << "leftConstraint: ";
        if (leftConstraint) {
          ss << leftConstraint->first.x() << ',' << leftConstraint->first.y() << ',' << leftConstraint->second;
        } else {
          ss << "NONE";
        }
        VLOG(1) << ss.str() << std::endl;
      }

      VLOG(1) << "Result of bendThatMfRoundTheRight:";
      if (optionalProjectedRightPoint) {
        VLOG(1) << "optionalProjectedRightPoint: " << optionalProjectedRightPoint->x() << ',' << optionalProjectedRightPoint->y();
      } else {
        VLOG(1) << "optionalProjectedRightPoint: NONE";
      }
      {
        std::stringstream ss;
        ss << "projectedRightIndex: ";
        if (projectedRightIndex) {
          ss << *projectedRightIndex;
        } else {
          ss << "NONE";
        }
        VLOG(1) << ss.str();
      }
      {
        std::stringstream ss;
        ss << "rightConstraint: ";
        if (rightConstraint) {
          ss << rightConstraint->first.x() << ',' << rightConstraint->first.y() << ',' << rightConstraint->second;
        } else {
          ss << "NONE";
        }
        VLOG(1) << ss.str() << std::endl;
      }
      if (!optionalProjectedLeftPoint) {
        throw std::runtime_error("[LeftLeft, RightRight, (Left)] Not yet handling optional");
      }
      if (!optionalProjectedRightPoint) {
        throw std::runtime_error("[LeftLeft, RightRight, (Right)] Not yet handling optional");
      }
      Vector projectedLeftPoint = *optionalProjectedLeftPoint;
      Vector projectedRightPoint = *optionalProjectedRightPoint;

      const bool successorIsBlockedByIntervalRight = [&](){
        if (!currentInterval.leftInterval()) {
          VLOG(1) << "No left interval";
          return false;
        }
        if (!currentInterval.rightInterval()) {
          VLOG(1) << "No right interval";
          return false;
        }
        if (!currentInterval.intervalLeftIsConstraintVertex()) {
          VLOG(1) << "Interval left is not a constraint vertex";
          return false;
        }
        if (!currentInterval.intervalRightIsConstraintVertex()) {
          VLOG(1) << "Interval right is not a constraint vertex";
          return false;
        }
        const LineSegment &leftInterval = *currentInterval.leftInterval();
        const LineSegment &rightInterval = *currentInterval.rightInterval();
        const double distanceToLeft = math::distanceSquared(leftInterval.first, leftInterval.second);
        const double distanceToRight = math::distanceSquared(rightInterval.first, rightInterval.second);
        if (distanceToRight < distanceToLeft) {
          VLOG(1) << "Right is closer, check for intersection";
          Vector leftIp0, leftIp1;
          const int leftIntervalIntersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, leftInterval.second, currentInterval.rightPoint, agentRadius_, &leftIp0, &leftIp1);
          if (leftIntervalIntersectionCount == 0) {
            VLOG(1) << "No intersection";
            return false;
          }
          VLOG(1) << "Intersection 1 at " << absl::StreamFormat("(%.20f,%.20f)", leftIp0.x(), leftIp0.y());
          if (leftIntervalIntersectionCount > 1) {
            VLOG(1) << "Intersection 2 at " << absl::StreamFormat("(%.20f,%.20f)", leftIp1.x(), leftIp1.y());
          }
          const bool leftActuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, leftInterval.second, currentInterval.rightPoint, agentRadius_, leftIntervalIntersectionCount, leftIp0, leftIp1);
          VLOG(1) << absl::StreamFormat("Left actually intersected? %v", leftActuallyIntersected);
          if (!leftActuallyIntersected) {
            return false;
          }
          // Confirmed the left interval intersects with right of currentInterval.
          // The right interval is not long enough, extend it to the successor edge.
          double iv0, iv1;
          const auto intersectionResult = math::intersectForIntervals(rightInterval.first, rightInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &iv0, &iv1);
          if (intersectionResult == math::IntersectionResult::kInfinite) {
            throw std::runtime_error("Not expecting infinite interesections");
          }
          const Vector extendedRightIntervalEnd = rightInterval.first + (rightInterval.second - rightInterval.first) * iv0;
          Vector rightIp0, rightIp1;
          const int rightIntervalIntersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, extendedRightIntervalEnd, currentInterval.leftPoint, agentRadius_, &rightIp0, &rightIp1);
          if (rightIntervalIntersectionCount == 0) {
            VLOG(1) << "No intersection";
            return false;
          }
          VLOG(1) << "Intersection 1 at " << absl::StreamFormat("(%.20f,%.20f)", rightIp0.x(), rightIp0.y());
          if (rightIntervalIntersectionCount > 1) {
            VLOG(1) << "Intersection 2 at " << absl::StreamFormat("(%.20f,%.20f)", rightIp1.x(), rightIp1.y());
          }
          const bool rightActuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, extendedRightIntervalEnd, currentInterval.leftPoint, agentRadius_, rightIntervalIntersectionCount, rightIp0, rightIp1);
          VLOG(1) << absl::StreamFormat("Right actually intersected? %v", rightActuallyIntersected);
          if (!rightActuallyIntersected) {
            return false;
          }
          // Confirmed the right interval intersects with left of currentInterval.
          VLOG(1) << "Seems to be our case";
          return true;
        }
        return false;
      }();
      const bool successorIsBlockedByIntervalLeft = [&](){
        if (!currentInterval.leftInterval()) {
          VLOG(1) << "No left interval";
          return false;
        }
        if (!currentInterval.rightInterval()) {
          VLOG(1) << "No right interval";
          return false;
        }
        if (!currentInterval.intervalLeftIsConstraintVertex()) {
          VLOG(1) << "Interval left is not a constraint vertex";
          return false;
        }
        if (!currentInterval.intervalRightIsConstraintVertex()) {
          VLOG(1) << "Interval right is not a constraint vertex";
          return false;
        }
        const LineSegment &leftInterval = *currentInterval.leftInterval();
        const LineSegment &rightInterval = *currentInterval.rightInterval();
        const double distanceToLeft = math::distanceSquared(leftInterval.first, leftInterval.second);
        const double distanceToRight = math::distanceSquared(rightInterval.first, rightInterval.second);
        if (distanceToLeft < distanceToRight) {
          VLOG(1) << "Left is closer, check for intersection";
          Vector rightIp0, rightIp1;
          const int rightIntervalIntersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, rightInterval.second, currentInterval.leftPoint, agentRadius_, &rightIp0, &rightIp1);
          if (rightIntervalIntersectionCount == 0) {
            VLOG(1) << "No intersection";
            return false;
          }
          VLOG(1) << "Intersection 1 at " << absl::StreamFormat("(%.20f,%.20f)", rightIp0.x(), rightIp0.y());
          if (rightIntervalIntersectionCount > 1) {
            VLOG(1) << "Intersection 2 at " << absl::StreamFormat("(%.20f,%.20f)", rightIp1.x(), rightIp1.y());
          }
          const bool rightActuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, rightInterval.second, currentInterval.leftPoint, agentRadius_, rightIntervalIntersectionCount, rightIp0, rightIp1);
          VLOG(1) << absl::StreamFormat("Right actually intersected? %v", rightActuallyIntersected);
          if (!rightActuallyIntersected) {
            return false;
          }
          // Confirmed the right interval intersects with left of currentInterval.
          // The left interval is not long enough, extend it to the successor edge.
          double iv0, iv1;
          const auto intersectionResult = math::intersectForIntervals(leftInterval.first, leftInterval.second, successorEdgeLeftPoint, successorEdgeRightPoint, &iv0, &iv1);
          if (intersectionResult == math::IntersectionResult::kInfinite) {
            throw std::runtime_error("Not expecting infinite interesections");
          }
          const Vector extendedLeftIntervalEnd = leftInterval.first + (leftInterval.second - leftInterval.first) * iv0;
          Vector leftIp0, leftIp1;
          const int leftIntervalIntersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, extendedLeftIntervalEnd, currentInterval.rightPoint, agentRadius_, &leftIp0, &leftIp1);
          if (leftIntervalIntersectionCount == 0) {
            VLOG(1) << "No intersection";
            return false;
          }
          VLOG(1) << "Intersection 1 at " << absl::StreamFormat("(%.20f,%.20f)", leftIp0.x(), leftIp0.y());
          if (leftIntervalIntersectionCount > 1) {
            VLOG(1) << "Intersection 2 at " << absl::StreamFormat("(%.20f,%.20f)", leftIp1.x(), leftIp1.y());
          }
          const bool leftActuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, extendedLeftIntervalEnd, currentInterval.rightPoint, agentRadius_, leftIntervalIntersectionCount, leftIp0, leftIp1);
          VLOG(1) << absl::StreamFormat("Left actually intersected? %v", leftActuallyIntersected);
          if (!leftActuallyIntersected) {
            return false;
          }
          // Confirmed the left interval intersects with right of currentInterval.
          VLOG(1) << "Seems to be our case";
          return true;
        }
        return false;
      }();
      if (successorIsBlockedByIntervalRight) {
        // We're here in this case because the  left interval intersects with the right constraint of our interval (before getting to our current entry edge) and
        //                                 the right interval intersects with the  left constraint of our interval (before getting to the successor edge).
        // Create an interval with the right constraint of our interval as the root & right, left constraint will be the left.
        //  Note that this interval does not say anything about the successor.
        IntervalType successorInterval(currentState);
        successorInterval.setRoot(currentInterval.rightPoint, *currentInterval.rightIndex, AngleDirection::kClockwise);
        successorInterval.setLeft(currentInterval.leftPoint, *currentInterval.leftIndex);
        successorInterval.setRight(currentInterval.rightPoint, *currentInterval.rightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting to have right interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      } else if (successorIsBlockedByIntervalLeft) {
        // We're here in this case because the right interval intersects with the  left constraint of our interval (before getting to our current entry edge) and
        //                                 the  left interval intersects with the right constraint of our interval (before getting to the successor edge).
        // Create an interval with the left constraint of our interval as the root & left, right constraint will be the right.
        //  Note that this interval does not say anything about the successor.
        IntervalType successorInterval(currentState);
        successorInterval.setRoot(currentInterval.leftPoint, *currentInterval.leftIndex, AngleDirection::kCounterclockwise);
        successorInterval.setLeft(currentInterval.leftPoint, *currentInterval.leftIndex);
        successorInterval.setRight(currentInterval.rightPoint, *currentInterval.rightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting to have left interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
      } else {
        if (intervalIsClosed(currentInterval, successorEdgeLeftPoint, successorEdgeRightPoint, true, true)) {
          VLOG(1) << "Cannot get past vertex to get to successor edge";
          VLOG(1) << "leftConstraint? " << (leftConstraint ? "ye" : "no") << ", rightConstraint? " << (rightConstraint ? "ye" : "no");
        } else {
          VLOG(1) << "Interval is not closed, going to directly visible successor";
          IntervalType successorInterval1(successorState);
          successorInterval1.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
          successorInterval1.setLeft(projectedLeftPoint, projectedLeftIndex);
          successorInterval1.setRight(projectedRightPoint, projectedRightIndex);
          successorInterval1.costToRoot = currentInterval.costToRoot;
          pushSuccessor(&currentInterval, successorInterval1, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);

          if (leftConstraint) {
            VLOG(1) << "leftConstraint";
            // Left of this interval is a vertex, this will create a second, non-observable successor.
            IntervalType successorInterval2(successorState);
            successorInterval2.setRoot(leftConstraint->first, leftConstraint->second, AngleDirection::kCounterclockwise);
            successorInterval2.setLeft(successorEdgeLeftPoint, successorEdgeLeftIndex);
            successorInterval2.setRight(projectedLeftPoint, projectedLeftIndex);
            // Updated root; update cost to get to root.
            const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, leftConstraint->first, AngleDirection::kCounterclockwise, agentRadius_);
            successorInterval2.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
            pushSuccessor(&currentInterval, successorInterval2, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
          }

          if (rightConstraint) {
            VLOG(1) << "rightConstraint";
            // Right of this interval is a vertex, this will create a third, non-observable successor.
            IntervalType successorInterval3(successorState);
            successorInterval3.setRoot(rightConstraint->first, rightConstraint->second, AngleDirection::kClockwise);
            successorInterval3.setLeft(projectedRightPoint, projectedRightIndex);
            successorInterval3.setRight(successorEdgeRightPoint, successorEdgeRightIndex);
            // Updated root; update cost to get to root.
            const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, rightConstraint->first, AngleDirection::kClockwise, agentRadius_);
            successorInterval3.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
            pushSuccessor(&currentInterval, successorInterval3, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
          }
        }
      }
    }
  }
}

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::doesRightIntervalIntersectWithLeftOfSuccessorEdge(const IntervalType &currentInterval,
bool successorLeftIsConstraintVertex,
IndexType successorEdgeLeftIndex,
const Vector &successorEdgeLeftPoint,
const Vector &successorEdgeRightPoint) const {
  // The following check looks for cases where the right interval intersects with the left constraint of the successor edge before reaching the successor edge.
  if (currentInterval.rightInterval() && currentInterval.rootIndex) {
    const LineSegment &rightInterval = *currentInterval.rightInterval();
    // It might be impossible to reach the successor edge from this interval.
    if (successorLeftIsConstraintVertex && successorEdgeLeftIndex != *currentInterval.rootIndex) {
      // Make sure that this interval edge is extended all the way to the successor edge.
      const State &currentState = currentInterval.state;
      std::optional<Vector> endOfIntervalForIntersectionTest;
      if (math::equal(rightInterval.first, rightInterval.second)) {
        // Right interval has no length.
        VLOG(1) << "Right interval has no length!";

        // We expect this point (the 0-length interval) to be where the current entry edge intersects with this constraint vertex.
        if (currentState.hasEntryEdgeIndex()) {
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          if (*currentInterval.rootIndex == currentEdgeRightIndex) {
            throw std::runtime_error("Not expecting them to be flipped");
            const auto point = math::extendLineSegmentToLength(currentEdgeRightPoint, currentEdgeLeftPoint, agentRadius_);
            if (!math::equal(point, rightInterval.first)) {
              throw std::runtime_error("0 Expecting interval to be on radius of the end of the entry edge");
            }
          } else if (*currentInterval.rootIndex == currentEdgeLeftIndex) {
            // We expect this point to be on the circle around the constraint vertex.
            if (!math::equal(math::distance(rightInterval.first, currentInterval.rootPoint), agentRadius_)) {
              throw std::runtime_error("Expecting point to be on root. Instead, distance is "+std::to_string(math::distance(rightInterval.first, currentInterval.rootPoint)));
            }

            // Turn the degenerate right interval into a real vector with a tangent.
            const auto [tangentStart, tangentEnd] = math::createVectorTangentToPointOnCircle(currentInterval.rootPoint, agentRadius_, rightInterval.first);
            VLOG(1) << "Created tangent is Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ')';
            endOfIntervalForIntersectionTest = tangentEnd;
          } else {
            const auto distanceFromRightIntervalToRoot = math::distance(currentInterval.rootPoint, rightInterval.first);
            if (distanceFromRightIntervalToRoot == math::distance(currentEdgeRightPoint, rightInterval.first)) {
              const auto [tangentStart, tangentEnd] = math::createVectorTangentToPointOnCircle(currentEdgeRightPoint, agentRadius_, rightInterval.first);
              VLOG(1) << "Created tangent is Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ')';
              endOfIntervalForIntersectionTest = tangentEnd;
            } else if (distanceFromRightIntervalToRoot == math::distance(currentEdgeLeftPoint, rightInterval.first)) {
              throw std::runtime_error("currentEdgeLeftPoint, rightInterval.first");
            } else {
              VLOG(1) << "Indices are " << *currentInterval.rootIndex << ',' << currentEdgeRightIndex << ',' << currentEdgeLeftIndex;
              throw std::runtime_error("[0] Expecting root to be one of current entry edge's vertices");
            }
          }
        } else {
          throw std::runtime_error("How can the interval have a root index but the current state doesnt have an entry edge?");
        }

        // TODO: This is a degenerate interval. For now, we skip. In the future, I think we should consider extending it.
        // Skip creating successors for now.
        if (!endOfIntervalForIntersectionTest) {
          throw std::runtime_error("Do we ever get here? I think every path is covered by exceptions; if not, this is a fallback");
          // Nothing else to do for this successor.
          return true;
        }
      }
      if (!endOfIntervalForIntersectionTest) {
        endOfIntervalForIntersectionTest = rightInterval.second;
      }
      std::optional<Vector> newEndOfInterval;
      double i1, i2;
      const auto intersectionResult = math::intersectForIntervals(rightInterval.first, *endOfIntervalForIntersectionTest, successorEdgeRightPoint, successorEdgeLeftPoint, &i1, &i2);
      if (intersectionResult == math::IntersectionResult::kOne) {
        // Already reaches to the successor edge.
        VLOG(1) << "New right end of interval created here";
        newEndOfInterval = *endOfIntervalForIntersectionTest;
      } else if (intersectionResult == math::IntersectionResult::kNone) {
        // Need to extend.
        if (i1 == std::numeric_limits<double>::infinity() && i2 == std::numeric_limits<double>::infinity()) {
          // Lines must be parallel but not overlapping.
          // If the right of our interval and the endpoint of the successor edge are agentRadius_ apart, then there is no intersection with a constraint
          if (math::lessThan(math::distance(*endOfIntervalForIntersectionTest, successorEdgeRightPoint), agentRadius_)) {
            throw std::runtime_error("i1="+std::to_string(i1)+" or i2="+std::to_string(i2)+" is inf [0]");
          }
        } else if (i1 == std::numeric_limits<double>::infinity() || i2 == std::numeric_limits<double>::infinity()) {
          throw std::runtime_error("i1="+std::to_string(i1)+" or i2="+std::to_string(i2)+" is inf [1]");
        } else {
          if (i1 > 0.0) {
            // Only create a new end for the interval if it is in front of the existing interval.
            newEndOfInterval = successorEdgeRightPoint + (successorEdgeLeftPoint-successorEdgeRightPoint) * i2;
          }
        }
      }
      if (newEndOfInterval) {
        Vector intersectionPoint1, intersectionPoint2;
        VLOG(1) << absl::StreamFormat("Want to check if Segment((%.20f,%.20f),(%.20f,%.20f)) intersects with circle at (%.20f,%.20f)", rightInterval.first.x(), rightInterval.first.y(), newEndOfInterval->x(), newEndOfInterval->y(), successorEdgeLeftPoint.x(), successorEdgeLeftPoint.y());
        const auto intersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, *newEndOfInterval, successorEdgeLeftPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
        const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, *newEndOfInterval, successorEdgeLeftPoint, agentRadius_, intersectionCount, intersectionPoint1, intersectionPoint2);
        if (actuallyIntersected) {
          // Right of interval intersects with the left vertex of the successor edge. Cannot only reach the edge if we can turn right around the constraint.
          if (!(currentInterval.rightIndex && isAConstraintVertexForState(currentState, *currentInterval.rightIndex))) {
            // Cannot turn right around the interval, cannot reach successor edge.
            // Nothing else to do for this successor.
            return true;
          }
        }
      }
    }
  }
  return false;
}

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::doesLeftIntervalIntersectWithRightOfSuccessorEdge(const IntervalType &currentInterval,
bool successorRightIsConstraintVertex,
IndexType successorEdgeRightIndex,
const Vector &successorEdgeLeftPoint,
const Vector &successorEdgeRightPoint) const {
  // The following check looks for cases where the left interval intersects with the right constraint of the successor edge before reaching the successor edge.
  if (currentInterval.leftInterval() && currentInterval.rootIndex) {
    const LineSegment &leftInterval = *currentInterval.leftInterval();
    // It might be impossible to reach the successor edge from this interval.
    if (successorRightIsConstraintVertex && successorEdgeRightIndex != *currentInterval.rootIndex) {
      // Make sure that this interval edge is extended all the way to the successor edge.
      const State &currentState = currentInterval.state;
      std::optional<Vector> endOfIntervalForIntersectionTest;
      if (math::equal(leftInterval.first, leftInterval.second)) {
        // Left interval has no length.
        VLOG(1) << "Left interval has no length!";

        // We expect this point (the 0-length interval) to be where the current entry edge intersects with this constraint vertex.
        if (currentState.hasEntryEdgeIndex()) {
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          if (*currentInterval.rootIndex == currentEdgeLeftIndex) {
            throw std::runtime_error("Not expecting them to be flipped");
            const auto point = math::extendLineSegmentToLength(currentEdgeLeftPoint, currentEdgeRightPoint, agentRadius_);
            if (!math::equal(point, leftInterval.first)) {
              throw std::runtime_error("2 Expecting interval to be on radius of the end of the entry edge");
            }
          } else if (*currentInterval.rootIndex == currentEdgeRightIndex) {
            // We expect this point to be on the circle around the constraint vertex.
            if (!math::equal(math::distance(leftInterval.first, currentInterval.rootPoint), agentRadius_)) {
              throw std::runtime_error("Expecting point to be on root. Instead, distance is "+std::to_string(math::distance(leftInterval.first, currentInterval.rootPoint)));
            }

            // Turn the degenerate left interval into a real vector with a tangent.
            const auto [tangentStart, tangentEnd] = math::createVectorTangentToPointOnCircle(currentInterval.rootPoint, agentRadius_, leftInterval.first);
            VLOG(1) << "Created tangent is Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ')';
            endOfIntervalForIntersectionTest = tangentEnd;
          } else {
            const auto distanceFromLeftIntervalToRoot = math::distance(currentInterval.rootPoint, leftInterval.first);
            if (distanceFromLeftIntervalToRoot == math::distance(currentEdgeLeftPoint, leftInterval.first)) {
              const auto [tangentStart, tangentEnd] = math::createVectorTangentToPointOnCircle(currentEdgeLeftPoint, agentRadius_, leftInterval.first);
              VLOG(1) << "Created tangent is Segment(" << '(' << tangentStart.x() << ',' << tangentStart.y() << ')' << ',' << '(' << tangentEnd.x() << ',' << tangentEnd.y() << ')' << ')';
              endOfIntervalForIntersectionTest = tangentEnd;
            } else if (distanceFromLeftIntervalToRoot == math::distance(currentEdgeRightPoint, leftInterval.first)) {
              throw std::runtime_error("currentEdgeRightPoint, leftInterval.first");
            } else {
              VLOG(1) << "Indices are " << *currentInterval.rootIndex << ',' << currentEdgeLeftIndex << ',' << currentEdgeRightIndex;
              throw std::runtime_error("[1] Expecting root to be one of current entry edge's vertices");
            }
          }
        } else {
          throw std::runtime_error("How can the interval have a root index but the current state doesnt have an entry edge?");
        }

        // TODO: This is a degenerate interval. For now, we skip. In the future, I think we should consider extending it.
        // Skip creating successors for now.
        if (!endOfIntervalForIntersectionTest) {
          throw std::runtime_error("Do we ever get here? I think every path is covered by exceptions; if not, this is a fallback");
          // Nothing else to do for this successor.
          return true;
        }
      }
      if (!endOfIntervalForIntersectionTest) {
        endOfIntervalForIntersectionTest = leftInterval.second;
      }
      std::optional<Vector> newEndOfInterval;
      double i1, i2;
      const auto intersectionResult = math::intersectForIntervals(leftInterval.first, *endOfIntervalForIntersectionTest, successorEdgeRightPoint, successorEdgeLeftPoint, &i1, &i2);
      if (intersectionResult == math::IntersectionResult::kOne) {
        // Already reaches to the successor edge.
        newEndOfInterval = *endOfIntervalForIntersectionTest;
      } else if (intersectionResult == math::IntersectionResult::kNone) {
        // Need to extend.
        if (i1 == std::numeric_limits<double>::infinity() && i2 == std::numeric_limits<double>::infinity()) {
          // Lines must be parallel but not overlapping.
          // If the left of our interval and the endpoint of the successor edge are agentRadius_ apart, then there is no intersection with a constraint
          if (math::lessThan(math::distance(*endOfIntervalForIntersectionTest, successorEdgeLeftPoint), agentRadius_)) {
            VLOG(1) << "End of left interval is not agentRadius_ away from the left of the successor edge";
            throw std::runtime_error("i1="+std::to_string(i1)+" or i2="+std::to_string(i2)+" is inf [2]");
          }
          // if (!(currentInterval.leftIndex && *currentInterval.leftIndex == successorEdgeLeftIndex)) {
        } else if (i1 == std::numeric_limits<double>::infinity() || i2 == std::numeric_limits<double>::infinity()) {
          throw std::runtime_error("i1="+std::to_string(i1)+" or i2="+std::to_string(i2)+" is inf [3]");
        } else {
          if (i1 > 0.0) {
            // Only create a new end for the interval if it is in front of the existing interval.
            newEndOfInterval = successorEdgeRightPoint + (successorEdgeLeftPoint-successorEdgeRightPoint) * i2;
          }
        }
      }
      if (newEndOfInterval) {
        Vector intersectionPoint1, intersectionPoint2;
        const auto intersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, *newEndOfInterval, successorEdgeRightPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
        const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, *newEndOfInterval, successorEdgeRightPoint, agentRadius_, intersectionCount, intersectionPoint1, intersectionPoint2);
        if (actuallyIntersected) {
          // Left of interval intersects with the right vertex of the successor edge. Cannot only reach the edge if we can turn left around the constraint.
          if (!(currentInterval.leftIndex && isAConstraintVertexForState(currentState, *currentInterval.leftIndex))) {
            // Cannot turn left around the interval, cannot reach successor edge.
            // Nothing else to do for this successor.
            return true;
          }
        }
      }
    }
  }
  return false;
}

template<typename NavmeshType>
internal::RelativePositionToInterval Pathfinder<NavmeshType>::calculateSuccessorLeftRelativeToInterval(const IntervalType &currentInterval,
IndexType currentEdgeLeftIndex,
const Vector &successorEdgeLeftPoint,
IndexType successorEdgeLeftIndex,
const Vector &successorEdgeRightPoint,
IndexType successorEdgeRightIndex,
bool successorLeftIsConstraintVertex) const {
  VLOG(1) << "Checking where the left of the successor edge compares to our interval";
  // Check where the left of the successor edge compares to our interval

  if (currentInterval.leftIndex && !currentInterval.intervalLeftIsConstraintVertex() && currentEdgeLeftIndex == successorEdgeLeftIndex) {
    // The left is not a constraint, is the left of the current edge, and is the left of the successor edge. It's the same point; it is neither left-of nor right-of.
    return internal::RelativePositionToInterval::kIsWithin;
  }

  std::optional<bool> leftVertexIsLeftOf;
  std::optional<bool> leftVertexIsRightOf;
  if (!currentInterval.leftIsRoot()) {
    VLOG(1) << " Left is not root";
    // We shouldn't calculate this if the left is the root.
    Vector pushedPoint;
    if (isAConstraintVertexForState(currentInterval.state, successorEdgeLeftIndex)) {
      pushedPoint = math::extendLineSegmentToLength(successorEdgeLeftPoint, successorEdgeRightPoint, agentRadius_);
    } else {
      pushedPoint = successorEdgeLeftPoint;
    }
    // What if the successor edge intersects with our constraint root?
    //  We differentiate between the two directions because we want to be able to use the first intersection point as the one that's relevant to our direction.
    if (currentInterval.rootDirection == AngleDirection::kCounterclockwise) {
      Vector intersectionPoint;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeRightPoint, successorEdgeLeftPoint, currentInterval.rootPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0) {
        // Only overwrite the `pushedPoint` if this new intersection is even more restrictive.
        if (math::distanceSquared(intersectionPoint, successorEdgeRightPoint) < math::distanceSquared(pushedPoint, successorEdgeRightPoint)) {
          VLOG(1) << absl::StreamFormat("Overwrite (%.20f,%.20f) with (%.20f,%.20f)", pushedPoint.x(), pushedPoint.y(), intersectionPoint.x(), intersectionPoint.y());
          pushedPoint = intersectionPoint;
        }
      }
    } else if (currentInterval.rootDirection == AngleDirection::kClockwise) {
      Vector intersectionPoint;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeLeftPoint, successorEdgeRightPoint, currentInterval.rootPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0) {
        // Only overwrite the `pushedPoint` if this new intersection is even more restrictive.
        if (math::distanceSquared(intersectionPoint, successorEdgeLeftPoint) < math::distanceSquared(pushedPoint, successorEdgeLeftPoint)) {
          VLOG(1) << absl::StreamFormat("Overwrite (%.20f,%.20f) with (%.20f,%.20f)", pushedPoint.x(), pushedPoint.y(), intersectionPoint.x(), intersectionPoint.y());
          pushedPoint = intersectionPoint;
        }
      }
    }
    VLOG(1) << absl::StreamFormat("Pushed point is at (%.20f,%.20f)", pushedPoint.x(), pushedPoint.y());
    auto [leftIntervalToSuccessorEntryEdgeStart,
          leftIntervalToSuccessorEntryEdgeEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, pushedPoint, AngleDirection::kNoDirection, agentRadius_);
    // Check if given line intersects with the left vertex of the successor edge.
    if (!currentInterval.leftIndex && successorLeftIsConstraintVertex) {
      Vector intersectionPoint;
      auto intersectionCount = math::lineSegmentIntersectsWithCircle(leftIntervalToSuccessorEntryEdgeStart, leftIntervalToSuccessorEntryEdgeEnd, successorEdgeLeftPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0 && leftIntervalToSuccessorEntryEdgeEnd != intersectionPoint) {
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting left interval");
        }
        // The intersection must be beyond the left interval
        const double leftIntervalLengthSquared = math::distanceSquared(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second);
        const double leftIntervalToIntersectionLengthSquared = math::distanceSquared(currentInterval.leftInterval()->first, intersectionPoint);
        if (leftIntervalToIntersectionLengthSquared > leftIntervalLengthSquared) {
          std::tie(leftIntervalToSuccessorEntryEdgeStart, leftIntervalToSuccessorEntryEdgeEnd) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
        }
      }
    }
    VLOG(1) << "Calling booberGobblin";
    const auto [tmpLeftVertexIsLeftOf, tmpLeftVertexIsRightOf] = booberGobblin(successorEdgeLeftIndex,
                                                                      successorEdgeRightIndex,
                                                                      currentInterval,
                                                                      leftIntervalToSuccessorEntryEdgeStart,
                                                                      leftIntervalToSuccessorEntryEdgeEnd);
    VLOG(1) << "Result is " << "tmpLeftVertexIsLeftOf: " << tmpLeftVertexIsLeftOf << ',' << "tmpLeftVertexIsRightOf: " << tmpLeftVertexIsRightOf;
    leftVertexIsLeftOf = tmpLeftVertexIsLeftOf;
    leftVertexIsRightOf = tmpLeftVertexIsRightOf;
  } else {
    VLOG(1) << "Left is root " << currentInterval.leftIsRoot();
    if (currentInterval.leftInterval()) {
      VLOG(1) << absl::StreamFormat("Have left interval: Segment((%.20f,%.20f),(%.20f,%.20f))", currentInterval.leftInterval()->first.x(), currentInterval.leftInterval()->first.y(), currentInterval.leftInterval()->second.x(), currentInterval.leftInterval()->second.y());
    } else {
      VLOG(1) << "No left interval";
    }
    if (successorEdgeLeftIndex != currentInterval.rootIndex) {
      // Successor left is not root, it might be outside the interval. Use the edge from root to successor edge left as a vector to check if the point is outside of the interval.
      if (math::distanceSquared(successorEdgeLeftPoint, currentInterval.rootPoint) < agentRadius_*agentRadius_) {
        leftVertexIsLeftOf = true;
        leftVertexIsRightOf = false;
      } else {
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting right interval");
        }
        Vector startOfSegment, endOfSegment;
        if (successorLeftIsConstraintVertex) {
          std::tie(startOfSegment, endOfSegment) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
        } else {
          std::tie(startOfSegment, endOfSegment) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeLeftPoint, AngleDirection::kNoDirection, agentRadius_);
        }
        VLOG(1) << "Calling booberGobblin";
        std::tie(leftVertexIsLeftOf, leftVertexIsRightOf) = booberGobblin(successorEdgeLeftIndex,
                                                                          successorEdgeRightIndex,
                                                                          currentInterval,
                                                                          startOfSegment,
                                                                          endOfSegment);
      }
    } else {
      // TODO: Comment why this is valid.
      leftVertexIsLeftOf = false;
      leftVertexIsRightOf = false;
    }
  }
  if (!leftVertexIsLeftOf) {
    // TODO: Previously, I think the value would be false, that's probably ok to keep for now.
    throw std::runtime_error("Reach end of function without setting, leftVertexIsLeftOf.");
  }
  if (!leftVertexIsRightOf) {
    // TODO: Previously, I think the value would be false, that's probably ok to keep for now.
    throw std::runtime_error("Reach end of function without setting, leftVertexIsRightOf.");
  }
  if (*leftVertexIsLeftOf == false && *leftVertexIsRightOf == false) {
    return internal::RelativePositionToInterval::kIsWithin;
  }
  if (*leftVertexIsLeftOf == true && *leftVertexIsRightOf == false) {
    return internal::RelativePositionToInterval::kIsLeftOf;
  }
  if (*leftVertexIsLeftOf == false && *leftVertexIsRightOf == true) {
    return internal::RelativePositionToInterval::kIsRightOf;
  }
  throw std::runtime_error("Invalid relative position calculateSuccessorLeftRelativeToInterval");
}

template<typename NavmeshType>
internal::RelativePositionToInterval Pathfinder<NavmeshType>::calculateSuccessorRightRelativeToInterval(const IntervalType &currentInterval,
IndexType currentEdgeRightIndex,
const Vector &successorEdgeLeftPoint,
IndexType successorEdgeLeftIndex,
const Vector &successorEdgeRightPoint,
IndexType successorEdgeRightIndex,
bool successorRightIsConstraintVertex) const {
  VLOG(1) << "Checking where the right of the successor edge compares to our interval";
  // Check where the right of the successor edge compares to our interval

  if (currentInterval.rightIndex && !currentInterval.intervalRightIsConstraintVertex() && currentEdgeRightIndex == successorEdgeRightIndex) {
    if (*currentInterval.rightIndex != currentEdgeRightIndex) {
      throw std::runtime_error("Expecting right of interval to be the same as the current entry edge right");
    }
    // The right is not a constraint, is the right of the current edge, and is the right of the successor edge. It's the same point; it is neither left-of nor right-of.
    return internal::RelativePositionToInterval::kIsWithin;
  }

  std::optional<bool> rightVertexIsLeftOf;
  std::optional<bool> rightVertexIsRightOf;
  if (!currentInterval.rightIsRoot()) {
    if (currentInterval.leftIntervalToCurrentEntryEdge()) {
      VLOG(1) << absl::StreamFormat("leftIntervalToCurrentEntryEdge: Segment((%.12f,%.12f),(%.12f,%.12f))", currentInterval.leftIntervalToCurrentEntryEdge()->first.x(),
                                                                                                            currentInterval.leftIntervalToCurrentEntryEdge()->first.y(),
                                                                                                            currentInterval.leftIntervalToCurrentEntryEdge()->second.x(),
                                                                                                            currentInterval.leftIntervalToCurrentEntryEdge()->second.y());
    }
    if (currentInterval.rightIntervalToCurrentEntryEdge()) {
      VLOG(1) << absl::StreamFormat("rightIntervalToCurrentEntryEdge: Segment((%.12f,%.12f),(%.12f,%.12f))", currentInterval.rightIntervalToCurrentEntryEdge()->first.x(),
                                                                                                             currentInterval.rightIntervalToCurrentEntryEdge()->first.y(),
                                                                                                             currentInterval.rightIntervalToCurrentEntryEdge()->second.x(),
                                                                                                             currentInterval.rightIntervalToCurrentEntryEdge()->second.y());
    }
    // We shouldn't calculate this if the right is the root.
    Vector pushedPoint;
    if (isAConstraintVertexForState(currentInterval.state, successorEdgeRightIndex)) {
      pushedPoint = math::extendLineSegmentToLength(successorEdgeRightPoint, successorEdgeLeftPoint, agentRadius_);
    } else {
      pushedPoint = successorEdgeRightPoint;
    }
    // What if the successor edge intersects with our constraint root?
    //  We differentiate between the two directions because we want to be able to use the first intersection point as the one that's relevant to our direction.
    if (currentInterval.rootDirection == AngleDirection::kCounterclockwise) {
      Vector intersectionPoint;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeRightPoint, successorEdgeLeftPoint, currentInterval.rootPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0) {
        // Only overwrite the `pushedPoint` if this new intersection is even more restrictive.
        if (math::distanceSquared(intersectionPoint, successorEdgeRightPoint) < math::distanceSquared(pushedPoint, successorEdgeRightPoint)) {
          VLOG(1) << absl::StreamFormat("Overwrite (%.20f,%.20f) with (%.20f,%.20f)", pushedPoint.x(), pushedPoint.y(), intersectionPoint.x(), intersectionPoint.y());
          pushedPoint = intersectionPoint;
        }
      }
    } else if (currentInterval.rootDirection == AngleDirection::kClockwise) {
      Vector intersectionPoint;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(successorEdgeLeftPoint, successorEdgeRightPoint, currentInterval.rootPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0) {
        // Only overwrite the `pushedPoint` if this new intersection is even more restrictive.
        if (math::distanceSquared(intersectionPoint, successorEdgeLeftPoint) < math::distanceSquared(pushedPoint, successorEdgeLeftPoint)) {
          VLOG(1) << absl::StreamFormat("Overwrite (%.20f,%.20f) with (%.20f,%.20f)", pushedPoint.x(), pushedPoint.y(), intersectionPoint.x(), intersectionPoint.y());
          pushedPoint = intersectionPoint;
        }
      }
    }
    auto [rightIntervalToSuccessorEntryEdgeStart,
          rightIntervalToSuccessorEntryEdgeEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, pushedPoint, AngleDirection::kNoDirection, agentRadius_);
    // Check if given line intersects with the right vertex of the successor edge.
    if (!currentInterval.rightIndex && successorRightIsConstraintVertex) {
      Vector intersectionPoint;
      auto intersectionCount = math::lineSegmentIntersectsWithCircle(rightIntervalToSuccessorEntryEdgeStart, rightIntervalToSuccessorEntryEdgeEnd, successorEdgeRightPoint, agentRadius_, &intersectionPoint);
      if (intersectionCount > 0 && rightIntervalToSuccessorEntryEdgeEnd != intersectionPoint) {
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting right interval");
        }
        // The intersection must be beyond the right interval
        const double rightIntervalLengthSquared = math::distanceSquared(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second);
        const double rightIntervalToIntersectionLengthSquared = math::distanceSquared(currentInterval.rightInterval()->first, intersectionPoint);
        if (rightIntervalToIntersectionLengthSquared > rightIntervalLengthSquared) {
          std::tie(rightIntervalToSuccessorEntryEdgeStart, rightIntervalToSuccessorEntryEdgeEnd) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
        }
      }
    }
    const auto [tmpRightVertexIsLeftOf, tmpRightVertexIsRightOf] = booberGobblin(successorEdgeLeftIndex,
                                                                        successorEdgeRightIndex,
                                                                        currentInterval,
                                                                        rightIntervalToSuccessorEntryEdgeStart,
                                                                        rightIntervalToSuccessorEntryEdgeEnd);
    rightVertexIsLeftOf = tmpRightVertexIsLeftOf;
    rightVertexIsRightOf = tmpRightVertexIsRightOf;
  } else {
    if (successorEdgeRightIndex != currentInterval.rootIndex) {
      // Successor right is not root, it might be outside the interval. Use the edge from root to successor edge right as a vector to check if the point is outside of the interval.
      if (math::distanceSquared(successorEdgeRightPoint, currentInterval.rootPoint) < agentRadius_*agentRadius_) {
        rightVertexIsLeftOf = false;
        rightVertexIsRightOf = true;
      } else {
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting leftInterval interval");
        }
        Vector startOfSegment, endOfSegment;
        if (successorRightIsConstraintVertex) {
          std::tie(startOfSegment, endOfSegment) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
        } else {
          std::tie(startOfSegment, endOfSegment) = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, successorEdgeRightPoint, AngleDirection::kNoDirection, agentRadius_);
        }
        VLOG(1) << "Calling booberGobblin";
        std::tie(rightVertexIsLeftOf, rightVertexIsRightOf) = booberGobblin(successorEdgeLeftIndex,
                                                                            successorEdgeRightIndex,
                                                                            currentInterval,
                                                                            startOfSegment,
                                                                            endOfSegment);
      }
    } else {
      // TODO: Comment why this is valid.
      rightVertexIsLeftOf = false;
      rightVertexIsRightOf = false;
    }
  }
  if (!rightVertexIsLeftOf) {
    // TODO: Previously, I think the value would be false, that's probably ok to keep for now.
    throw std::runtime_error("Reach end of function without setting, rightVertexIsLeftOf.");
  }
  if (!rightVertexIsRightOf) {
    // TODO: Previously, I think the value would be false, that's probably ok to keep for now.
    throw std::runtime_error("Reach end of function without setting, rightVertexIsRightOf.");
  }
  if (*rightVertexIsLeftOf == false && *rightVertexIsRightOf == false) {
    return internal::RelativePositionToInterval::kIsWithin;
  }
  if (*rightVertexIsLeftOf == true && *rightVertexIsRightOf == false) {
    return internal::RelativePositionToInterval::kIsLeftOf;
  }
  if (*rightVertexIsLeftOf == false && *rightVertexIsRightOf == true) {
    return internal::RelativePositionToInterval::kIsRightOf;
  }
  throw std::runtime_error("Invalid relative position calculateSuccessorRightRelativeToInterval");
}

// This function checks if it's possible to get through the interval.
// @param checkIfIntersectWithLeftConstraint should be true if we want to check if it's possible to get to the end of rightInterval within this interval
// @param checkIfIntersectWithRightConstraint should be true if we want to check if it's possible to get to the end of leftInterval within this interval
template<typename NavmeshType>
bool Pathfinder<NavmeshType>::intervalIsClosed(const IntervalType &currentInterval, const Vector &successorEdgeLeftPoint, const Vector &successorEdgeRightPoint, bool checkIfIntersectWithLeftConstraint, bool checkIfIntersectWithRightConstraint) const {
  VLOG(1) << "Enter intervalIsClosed";
  internal::RAIIPrinter raiiPrinter("Exit intervalIsClosed");
  if (checkIfIntersectWithLeftConstraint && currentInterval.rightInterval() && currentInterval.intervalLeftIsConstraintVertex()) {
    // Check if the right interval intersects with the left constraint vertex.
    // First, extend the line segment so that it is long enough to hit the vertex.
    const LineSegment &rightInterval = *currentInterval.rightInterval();
    auto newRightIntervalEnd = math::extendLineSegmentToLength(rightInterval.first, rightInterval.second, math::distance(rightInterval.first, currentInterval.leftPoint));
    // If the start of the right interval is inside the radius of the left interval
    VLOG(1) << "Extended right interval is Segment(" << '(' << rightInterval.first.x() << ',' << rightInterval.first.y() << ')' << ',' << '(' << newRightIntervalEnd.x() << ',' << newRightIntervalEnd.y() << ')' << ")";
    Vector intersectionPoint1, intersectionPoint2;
    const int circleIntersectionCount = math::lineSegmentIntersectsWithCircle(rightInterval.first, newRightIntervalEnd, currentInterval.leftPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
    // Check if the segment intersects with the circle, and isnt only a tangential intersection.
    const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(rightInterval.first, newRightIntervalEnd, currentInterval.leftPoint, agentRadius_, circleIntersectionCount, intersectionPoint1, intersectionPoint2);
    if (actuallyIntersected) {
      std::stringstream ss;
      ss << "Right interval intersects with left interval constraint vertex (" << *currentInterval.leftIndex << ") " << circleIntersectionCount << " time" << (circleIntersectionCount == 1 ? "" : "s") << "!";
      if (circleIntersectionCount > 0) {
        ss << " Intersection 1: " << intersectionPoint1.x() << ',' << intersectionPoint1.y();
        if (circleIntersectionCount > 1) {
          ss << " Intersection 2: " << intersectionPoint2.x() << ',' << intersectionPoint2.y();
        }
      }
      VLOG(1) << ss.str();
      // If this line does not intersect with the successor edge before intersecting with the circle, then the successor cannot be reached.
      double t1, t2;
      auto intersectionResult = math::intersectForIntervals(rightInterval.first, intersectionPoint1, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      if (intersectionResult != math::IntersectionResult::kOne) {
        if (intersectionResult == math::IntersectionResult::kInfinite) {
          throw std::runtime_error("Infinite intersection!");
        }
        if (t1 == std::numeric_limits<double>::infinity() && t2 == std::numeric_limits<double>::infinity()) {
          VLOG(1) << "Lines are parallel but not touching";
        } else if (math::lessThan(t1, 0.0)) {
          VLOG(1) << "Line intersects with left vertex, but this ray never intersects with the successor edge.";
        } else {
          // Cannot reach successor edge.
          VLOG(1) << "Cannot reach successor edge, hitting left Constraint Vertex, t1:" << t1 << ", t2:" << t2;
          return true;
        }
      } else {
        VLOG(1) << "[rightInterval&LeftConstraint] We can reach the successor edge though";
      }
    } else {
      VLOG(1) << "[rightInterval&LeftConstraint] Does not intersect with circle";
    }
  }
  if (checkIfIntersectWithRightConstraint && currentInterval.leftInterval() && currentInterval.intervalRightIsConstraintVertex()) {
    // Check if the left interval intersects with the right constraint vertex.
    // First, extend the line segment so that it is long enough to hit the vertex.
    const LineSegment &leftInterval = *currentInterval.leftInterval();
    auto newLeftIntervalEnd = math::extendLineSegmentToLength(leftInterval.first, leftInterval.second, math::distance(leftInterval.first, currentInterval.rightPoint));
    Vector intersectionPoint1, intersectionPoint2;
    const int circleIntersectionCount = math::lineSegmentIntersectsWithCircle(leftInterval.first, newLeftIntervalEnd, currentInterval.rightPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
    VLOG(1) << "Extended left interval is Segment(" << '(' << leftInterval.first.x() << ',' << leftInterval.first.y() << ')' << ',' << '(' << newLeftIntervalEnd.x() << ',' << newLeftIntervalEnd.y() << ')' << ")";
    // Check if the segment intersects with the circle, and isnt only a tangential intersection.
    const bool actuallyIntersected = internal::lineActuallyIntersectedWithCircle(leftInterval.first, newLeftIntervalEnd, currentInterval.rightPoint, agentRadius_, circleIntersectionCount, intersectionPoint1, intersectionPoint2);
    if (actuallyIntersected) {
      VLOG(1) << "Left interval intersects with right interval constraint vertex (" << circleIntersectionCount << " time(s))!";
      // If this line does not intersect with the successor edge before intersecting with the circle, then the successor cannot be reached.
      double t1, t2;
      auto intersectionResult = math::intersectForIntervals(leftInterval.first, intersectionPoint1, successorEdgeLeftPoint, successorEdgeRightPoint, &t1, &t2);
      if (intersectionResult != math::IntersectionResult::kOne) {
        if (intersectionResult == math::IntersectionResult::kInfinite) {
          throw std::runtime_error("Infinite intersection!");
        }
        if (t1 == std::numeric_limits<double>::infinity() && t2 == std::numeric_limits<double>::infinity()) {
          VLOG(1) << "Lines are parallel but not touching";
        } else if (math::lessThan(t1, 0.0)) {
          VLOG(1) << "Line intersects with right vertex, but this ray never intersects with the successor edge.";
        } else {
          // Cannot reach successor edge.
          VLOG(1) << "Cannot reach successor edge, hitting right Constraint Vertex, t1:" << t1 << ", t2:" << t2;
          return true;
        }
      } else {
        VLOG(1) << "[leftInterval&RightConstraint] We can reach the successor edge though";
      }
    } else {
      VLOG(1) << "[leftInterval&RightConstraint] Does not intersect with circle";
    }
  }
  return false;
}

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::canFitThroughEdge(const State &currentState, IndexType edgeIndex) const {
  const auto [v1Index, v2Index] = navmesh_.getEdgeVertexIndices(edgeIndex);
  const bool v1IsConstraint = isAConstraintVertexForState(currentState, v1Index);
  const bool v2IsConstraint = isAConstraintVertexForState(currentState, v2Index);
  if (v1IsConstraint && v2IsConstraint) {
    // Both vertices are constraints.
    const auto [v1, v2] = navmesh_.getEdge(edgeIndex);
    if (math::distanceSquared(v1, v2) < 4*agentRadius_*agentRadius_) {
      // We cannot fit through the edge.
      return false;
    }
  } else if (v1IsConstraint || v2IsConstraint) {
    // One of the vertices is a constraint.
    const auto [v1, v2] = navmesh_.getEdge(edgeIndex);
    if (math::distanceSquared(v1, v2) < agentRadius_*agentRadius_) {
      // We cannot fit through the edge.
      return false;
    }
  }
  return true;
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::buildLeftIntervals(IntervalType &currentInterval) const {
  std::optional<LineSegment> leftInterval;
  std::optional<LineSegment> leftIntervalToCurrentEntryEdge;
  if (!currentInterval.leftIsRoot()) {
    const State &currentState = currentInterval.state;
    if (currentInterval.rootIndex && !currentInterval.intervalLeftIsConstraintVertex() && math::distanceSquared(currentInterval.rootPoint, currentInterval.leftPoint) < agentRadius_*agentRadius_) {
      // Probably should make the intersection of the current interval's root and the current entry edge a 0-length interval
      if (currentState.hasEntryEdgeIndex()) {
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        Vector ip0;
        int intersectionCount = math::lineSegmentIntersectsWithCircle(currentEdgeLeftPoint, currentEdgeRightPoint, currentInterval.rootPoint, agentRadius_, &ip0);
        if (intersectionCount == 0) {
          throw std::runtime_error("Expecting an intersection");
        }
        leftInterval.emplace(std::make_pair(ip0,ip0));
      } else {
        throw std::runtime_error("Expecting to have an entry edge in this case");
      }
    } else {
      VLOG(1) << absl::StreamFormat("Creating circle conscious line from (%.12f,%.12f) with direction %s to (%.12f,%.12f) with direction %s", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), toString(currentInterval.rootDirection), currentInterval.leftPoint.x(), currentInterval.leftPoint.y(), toString(currentInterval.leftDirection()));
      leftInterval = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentInterval.leftPoint, currentInterval.leftDirection(), agentRadius_);
      if (currentState.hasEntryEdgeIndex()) {
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        Vector pushedPoint;
        if (isAConstraintVertexForState(currentState, currentEdgeLeftIndex)) {
          pushedPoint = math::extendLineSegmentToLength(currentEdgeLeftPoint, currentEdgeRightPoint, agentRadius_);
          if (currentInterval.rootDirection != AngleDirection::kNoDirection && math::distance(pushedPoint, currentInterval.rootPoint) < agentRadius_) {
            VLOG(1) << "Pushed point is inside of our radius!";
            // Instead, lets move the pushed point to the point where this edge intersects with our circle.
            Vector i0, i1;
            // We intentionally choose the edge direction right->left for intersection. If there are two intersection points, we will use the right one (since that one happens first).
            const auto intersectionCount = math::lineSegmentIntersectsWithCircle(currentEdgeRightPoint, currentEdgeLeftPoint, currentInterval.rootPoint, agentRadius_, &i0, &i1);
            VLOG(1) << "Moving intersection point from " << absl::StreamFormat("(%.12f,%.12f) to (%.12f,%.12f)",pushedPoint.x(), pushedPoint.y(), i0.x(), i0.y());
            pushedPoint = i0;
            // TODO: At this point, the following call of createCircleConsciousLine is wasteful, since the point is on the circumference of the circle by definition.
          }
        } else {
          pushedPoint = currentEdgeLeftPoint;
        }
        VLOG(1) << absl::StreamFormat("Creating circle conscious line from (%.12f,%.12f) with direction %s to (%.12f,%.12f) with direction %s", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), toString(currentInterval.rootDirection), pushedPoint.x(), pushedPoint.y(), toString(AngleDirection::kNoDirection));
        leftIntervalToCurrentEntryEdge = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, pushedPoint, AngleDirection::kNoDirection, agentRadius_);
      }
    }
  }
  if (leftInterval) {
    VLOG(1) << "leftInterval: Segment(" << absl::StreamFormat("(%.10f,%.10f)",leftInterval->first.x(), leftInterval->first.y()) << ',' << absl::StreamFormat("(%.10f,%.10f)",leftInterval->second.x(), leftInterval->second.y()) << ')';
  } else {
    VLOG(1) << "No leftInterval";
  }
  if (leftIntervalToCurrentEntryEdge) {
    VLOG(1) << "leftIntervalToCurrentEntryEdge: Segment(" << '(' << leftIntervalToCurrentEntryEdge->first.x() << ',' << leftIntervalToCurrentEntryEdge->first.y() << ')' << ',' << '(' << leftIntervalToCurrentEntryEdge->second.x() << ',' << leftIntervalToCurrentEntryEdge->second.y() << ')' << ')';
  } else {
    VLOG(1) << "No leftIntervalToCurrentEntryEdge";
  }
  currentInterval.setLeftIntervals(leftInterval, leftIntervalToCurrentEntryEdge);
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::buildRightIntervals(IntervalType &currentInterval) const {
  std::optional<LineSegment> rightInterval;
  std::optional<LineSegment> rightIntervalToCurrentEntryEdge;
  if (!currentInterval.rightIsRoot()) {
    const State &currentState = currentInterval.state;
    if (currentInterval.rootIndex && !currentInterval.intervalRightIsConstraintVertex() && math::distanceSquared(currentInterval.rootPoint, currentInterval.rightPoint) < agentRadius_*agentRadius_) {
      // Probably should make the intersection of the current interval's root and the current entry edge a 0-length interval
      if (currentState.hasEntryEdgeIndex()) {
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        Vector ip0;
        int intersectionCount = math::lineSegmentIntersectsWithCircle(currentEdgeRightPoint, currentEdgeLeftPoint, currentInterval.rootPoint, agentRadius_, &ip0);
        if (intersectionCount == 0) {
          throw std::runtime_error("Expecting an intersection");
        }
        rightInterval.emplace(std::make_pair(ip0,ip0));
      } else {
        throw std::runtime_error("Expecting to have an entry edge in this case");
      }
    } else {
      // We shouldn't calculate this if the right is the root.
      VLOG(1) << absl::StreamFormat("Creating circle conscious line from (%.12f,%.12f) with direction %s to (%.12f,%.12f) with direction %s", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), toString(currentInterval.rootDirection), currentInterval.rightPoint.x(), currentInterval.rightPoint.y(), toString(currentInterval.rightDirection()));
      rightInterval = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentInterval.rightPoint, currentInterval.rightDirection(), agentRadius_);
      if (currentState.hasEntryEdgeIndex()) {
        const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
        Vector pushedPoint;
        if (isAConstraintVertexForState(currentState, currentEdgeRightIndex)) {
          pushedPoint = math::extendLineSegmentToLength(currentEdgeRightPoint, currentEdgeLeftPoint, agentRadius_);
          if (currentInterval.rootDirection != AngleDirection::kNoDirection && math::distance(pushedPoint, currentInterval.rootPoint) < agentRadius_) {
            VLOG(1) << "Pushed point is inside of our radius!";
            // Instead, lets move the pushed point to the point where this edge intersects with our circle.
            Vector i0, i1;
            // We intentionally choose the edge direction left->right for intersection. If there are two intersection points, we will use the left one (since that one happens first).
            const auto intersectionCount = math::lineSegmentIntersectsWithCircle(currentEdgeLeftPoint, currentEdgeRightPoint, currentInterval.rootPoint, agentRadius_, &i0, &i1);
            VLOG(1) << "Moving intersection point from " << absl::StreamFormat("(%.12f,%.12f) to (%.12f,%.12f)",pushedPoint.x(), pushedPoint.y(), i0.x(), i0.y());
            pushedPoint = i0;
            // TODO: At this point, the following call of createCircleConsciousLine is wasteful, since the point is on the circumference of the circle by definition.
          }
        } else {
          pushedPoint = currentEdgeRightPoint;
        }
        VLOG(1) << absl::StreamFormat("Creating circle conscious line from (%.12f,%.12f) with direction %s to (%.12f,%.12f) with direction %s", currentInterval.rootPoint.x(), currentInterval.rootPoint.y(), toString(currentInterval.rootDirection), pushedPoint.x(), pushedPoint.y(), toString(AngleDirection::kNoDirection));
        rightIntervalToCurrentEntryEdge = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, pushedPoint, AngleDirection::kNoDirection, agentRadius_);
      }
    }
  }
  if (rightInterval) {
    VLOG(1) << absl::StreamFormat("rightInterval: Segment((%.12f,%.12f),(%.12f,%.12f))", rightInterval->first.x(), rightInterval->first.y(), rightInterval->second.x(), rightInterval->second.y());
  } else {
    VLOG(1) << "No rightInterval";
  }
  if (rightIntervalToCurrentEntryEdge) {
    VLOG(1) << "rightIntervalToCurrentEntryEdge: Segment(" << '(' << rightIntervalToCurrentEntryEdge->first.x() << ',' << rightIntervalToCurrentEntryEdge->first.y() << ')' << ',' << '(' << rightIntervalToCurrentEntryEdge->second.x() << ',' << rightIntervalToCurrentEntryEdge->second.y() << ')' << ')';
  } else {
    VLOG(1) << "No rightIntervalToCurrentEntryEdge";
  }
  currentInterval.setRightIntervals(rightInterval, rightIntervalToCurrentEntryEdge);
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::handleGoalSuccessor(const IntervalType &currentInterval,
                                                  const State &successorState,
                                                  IntervalHeapType &intervalHeap,
                                                  IntervalSetType &visited,
                                                  IntervalSetType &pushed,
                                                  PreviousIntervalMapType &previous,
                                                  const Vector &goalPoint,
                                                  typename PathfindingResult::DebugAStarInfoType &debugAStarInfo) const {
  const State &currentState = currentInterval.state;
  VLOG(1) << "Is goal";
  const auto [rootToGoalLineStart, rootToGoalLineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, goalPoint, AngleDirection::kNoDirection, agentRadius_);
  VLOG(1) << "Line to goal is Segment(" <<'(' << rootToGoalLineStart.x() << ',' << rootToGoalLineStart.y() << ')' <<  ',' <<'(' << rootToGoalLineEnd.x() << ',' << rootToGoalLineEnd.y() << ')' <<  ')';
  (void)rootToGoalLineEnd; // This point is the goal point.

  std::optional<std::tuple<Vector, IndexType, AngleDirection>> constraintToTurnAround;
  if (currentInterval.rightIsRoot()) {
    // Does the line to the goal intersect with the left of the interval
    if (currentInterval.intervalLeftIsConstraintVertex()) {
      Vector ip0, ip1;
      const auto intersectResult = math::lineSegmentIntersectsWithCircle(rootToGoalLineStart, rootToGoalLineEnd, currentInterval.leftPoint, agentRadius_, &ip0, &ip1);
      if (internal::lineActuallyIntersectedWithCircle(rootToGoalLineStart, rootToGoalLineEnd, currentInterval.leftPoint, agentRadius_, intersectResult, ip0, ip1)) {
        VLOG(1) << "Whoa! Line-to-goal intersects with left of interval";
        // Goal cannot be reached directly. Need to create a successor interval.
        IntervalType successorInterval(currentState);
        successorInterval.setRoot(currentInterval.leftPoint, *currentInterval.leftIndex, AngleDirection::kCounterclockwise);
        successorInterval.setLeft(currentInterval.leftPoint, *currentInterval.leftIndex);
        successorInterval.setRight(currentInterval.rightPoint, *currentInterval.rightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.leftInterval()) {
          throw std::runtime_error("Expecting to have left interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
        // Nothing else to do for this successor.
        return;
      }
    }
    // Need to check if the goal is to the left of the interval.
    if (!currentInterval.leftInterval()) {
      throw std::runtime_error("Expecting left interval");
    }
    const LineSegment &leftInterval = *currentInterval.leftInterval();
    if (currentInterval.intervalLeftIsConstraintVertex() && math::equal(leftInterval.first, leftInterval.second)) {
      VLOG(1) << "Left interval is 0-length";
      const auto [tmpStart, tmpEnd] = createTempLeftIntervalForCrossProduct(currentInterval, leftInterval.first);
      VLOG(1) << "Created tmp interval is Segment(" << '(' << tmpStart.x() << ',' << tmpStart.y() << ')' << ',' << '(' << tmpEnd.x() << ',' << tmpEnd.y() << ')' << ")";
      if (math::lessThan(0.0, math::crossProductForSign(tmpStart, tmpEnd, rootToGoalLineStart, goalPoint))) {
        VLOG(1) << "Goal is to the left; going to need to turn around";
        // TODO: Turning around this contraint could take us out of the current triangle. If so, goal is not reachable.
        constraintToTurnAround.emplace(currentInterval.leftPoint, *currentInterval.leftIndex, AngleDirection::kCounterclockwise);
      } else {
        VLOG(1) << "Goal is not to the left of the left of the interval; we can go directly to it";
      }
    } else if (!currentInterval.intervalLeftIsConstraintVertex()) {
      // If the goal is to the left, we cannot reach it.
      if (math::lessThan(0.0, math::crossProductForSign(leftInterval.first, leftInterval.second, rootToGoalLineStart, goalPoint))) {
        VLOG(1) << "Goal is to the left of the left interval. Cannot reach";
        // Nothing else to do for this successor.
        return;
      }
    } else {
      // Goal can guaranteed be reached.
      VLOG(1) << "[1] Goal can be reached directly. leftIsRoot:" << currentInterval.leftIsRoot() << ", rightIsRoot:" << currentInterval.rightIsRoot();
    }
  } else if (currentInterval.leftIsRoot()) {
    // Does the line to the goal intersect with the right of the interval
    if (currentInterval.intervalRightIsConstraintVertex()) {
      Vector ip0, ip1;
      const auto intersectResult = math::lineSegmentIntersectsWithCircle(rootToGoalLineStart, rootToGoalLineEnd, currentInterval.rightPoint, agentRadius_, &ip0, &ip1);
      if (internal::lineActuallyIntersectedWithCircle(rootToGoalLineStart, rootToGoalLineEnd, currentInterval.rightPoint, agentRadius_, intersectResult, ip0, ip1)) {
        VLOG(1) << "Whoa! Line-to-goal intersects with right of interval";
        // Goal cannot be reached directly. Need to create a successor interval.
        IntervalType successorInterval(currentState);
        successorInterval.setRoot(currentInterval.rightPoint, *currentInterval.rightIndex, AngleDirection::kClockwise);
        successorInterval.setLeft(currentInterval.leftPoint, *currentInterval.leftIndex);
        successorInterval.setRight(currentInterval.rightPoint, *currentInterval.rightIndex);
        // Updated root; update cost to get to root.
        if (!currentInterval.rightInterval()) {
          throw std::runtime_error("Expecting to have right interval");
        }
        successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
        pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
        // Nothing else to do for this successor.
        return;
      }
    }
    // Need to check if the goal is to the right of the interval.
    if (!currentInterval.rightInterval()) {
      throw std::runtime_error("Expecting right interval");
    }
    const LineSegment &rightInterval = *currentInterval.rightInterval();
    if (currentInterval.intervalRightIsConstraintVertex() && math::equal(rightInterval.first, rightInterval.second)) {
      VLOG(1) << "Right interval is 0-length";
      const auto [tmpStart, tmpEnd] = createTempRightIntervalForCrossProduct(currentInterval, rightInterval.first);
      VLOG(1) << "Created tmp interval is Segment(" << '(' << tmpStart.x() << ',' << tmpStart.y() << ')' << ',' << '(' << tmpEnd.x() << ',' << tmpEnd.y() << ')' << ")";
      if (math::lessThan(0.0, math::crossProductForSign(rootToGoalLineStart, goalPoint, tmpStart, tmpEnd))) {
        VLOG(1) << "Goal is to the right; going to need to turn around";
        // TODO: Turning around this contraint could take us out of the current triangle. If so, goal is not reachable.
        constraintToTurnAround.emplace(currentInterval.rightPoint, *currentInterval.rightIndex, AngleDirection::kClockwise);
      } else {
        VLOG(1) << "Goal is not to the right of the right of the interval; we can go directly to it";
      }
    } else if (!currentInterval.intervalRightIsConstraintVertex()) {
      // If the goal is to the right, we cannot reach it.
      if (math::lessThan(0.0, math::crossProductForSign(rootToGoalLineStart, goalPoint, rightInterval.first, rightInterval.second))) {
        VLOG(1) << "Goal is to the right of the right interval. Cannot reach";
        // Nothing else to do for this successor.
        return;
      }
    } else {
      // Goal can guaranteed be reached.
      VLOG(1) << "[1] Goal can be reached directly. leftIsRoot:" << currentInterval.leftIsRoot() << ", rightIsRoot:" << currentInterval.rightIsRoot();
    }
  } else {
    // Neither side is the root.
    const auto [isLeftOf, isRightOf] = booberGobblin( /*successorEdgeLeftIndex=*/std::nullopt,
                                                      /*successorEdgeRightIndex=*/std::nullopt,
                                                      currentInterval,
                                                      rootToGoalLineStart,
                                                      rootToGoalLineEnd/*==goalPoint*/);
    VLOG(1) << "isLeftOf: " << isLeftOf << ", isRightOf: " << isRightOf;
    if (!isLeftOf && !isRightOf) {
      // Line to goal is within the interval; it can be reached.
      VLOG(1) << "Is neither left nor right, can be directly reached";
    } else if (isLeftOf && isRightOf) {
      // Interval crosses over itself. Cannot get to goal.
      // TODO: I'm not 100% sure that this is valid in all cases.
      //  It is in this case:
      //   movePathStart({1771.0,113.0});
      //   movePathGoal({1390.81113,69.94196}); // Farther than vertex 9
      // Nothing else to do for this successor.
      return;
    } else {
      // Cannot go directly to the goal, maybe we can turn around some constraint.
      VLOG(1) << "Cannot go directly to the goal, maybe we can turn around some constraint.";
      if (isLeftOf) {
        if (currentInterval.intervalLeftIsConstraintVertex()) {
          // What if we hit the right of the interval on our way to this left point?
          if (currentInterval.intervalRightIsConstraintVertex() && currentInterval.leftInterval() && currentInterval.rightInterval()) {
            Vector ip0, ip1;
            const int intersectionCount = math::lineSegmentIntersectsWithCircle(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second, currentInterval.rightPoint, agentRadius_, &ip0, &ip1);
            if (internal::lineActuallyIntersectedWithCircle(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second, currentInterval.rightPoint, agentRadius_, intersectionCount, ip0, ip1)) {
              // On the way to the left, we hit the right. We cannot reach the goal in one step. Push a new interval going to just the right of this interval.
              IntervalType successorInterval(currentState);
              successorInterval.setRoot(currentInterval.rightPoint, currentInterval.rightIndex, AngleDirection::kClockwise);
              successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
              successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
              // Nothing else to do for this successor.
              return;
            }
          }
          // TODO: It could be the case that turning around this vertex might force us outside of our triangle.
          // What if after turning around this constraint, we hit the right of the interval on the way to the goal?
          //  Push a new interval to left of the interval, still for the current state. A follow-up iteration will check for that
          if (!currentInterval.leftInterval()) {
            throw std::runtime_error("Expecting left interval");
          }
          IntervalType successorInterval(currentState);
          successorInterval.setRoot(currentInterval.leftPoint, currentInterval.leftIndex, AngleDirection::kCounterclockwise);
          successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
          successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
          successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
          pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
          // Nothing else to do for this successor.
          return;
        } else if (currentState.hasEntryEdgeIndex()) {
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          if (isAConstraintVertexForState(currentState, currentEdgeLeftIndex)) {
            const auto [startOfRootToLeftOfEntryEdge, endOfRootToLeftOfEntryEdge] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentEdgeLeftPoint, AngleDirection::kCounterclockwise, agentRadius_);
            if (!math::lessThan(math::crossProductForSign(startOfRootToLeftOfEntryEdge, endOfRootToLeftOfEntryEdge, currentInterval.leftInterval()->first, currentInterval.leftInterval()->second), 0.0)) {
              // Check if the path to the goal intersects with the left vertex of this edge.
              const auto intersectionCount = math::lineSegmentIntersectsWithCircle(rootToGoalLineStart, goalPoint, currentEdgeLeftPoint, agentRadius_);
              if (intersectionCount == 2) {
                VLOG(1) << "Ic=2";
                // The goal is on the other side of this entry edge's left vertex. We can turn around that.
                constraintToTurnAround.emplace(currentEdgeLeftPoint, currentEdgeLeftIndex, AngleDirection::kCounterclockwise);
              } else if (intersectionCount == 1) {
                // This happens if the goal is inside the radius of the vertex.
                throw std::runtime_error("Not expecting 1 intersection to be possible. Case 0");
              }
            }
          }
        }
      } else if (isRightOf) {
        if (currentInterval.intervalRightIsConstraintVertex()) {
          // What if we hit the left of the interval on our way to this right point?
          if (currentInterval.intervalLeftIsConstraintVertex() && currentInterval.leftInterval() && currentInterval.rightInterval()) {
            Vector ip0, ip1;
            VLOG(1) << "Checking intersection of " << absl::StreamFormat("Segment((%.20f,%.20f),(%.20f,%.20f)) against (%.20f,%.20f)", currentInterval.rightInterval()->first.x(), currentInterval.rightInterval()->first.y(), currentInterval.rightInterval()->second.x(), currentInterval.rightInterval()->second.y(), currentInterval.leftPoint.x(), currentInterval.leftPoint.y());
            const int intersectionCount = math::lineSegmentIntersectsWithCircle(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second, currentInterval.leftPoint, agentRadius_, &ip0, &ip1);
            if (internal::lineActuallyIntersectedWithCircle(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second, currentInterval.leftPoint, agentRadius_, intersectionCount, ip0, ip1)) {
              // On the way to the right, we hit the left. We cannot reach the goal in one step. Push a new interval going to just the left of this interval.
              IntervalType successorInterval(currentState);
              successorInterval.setRoot(currentInterval.leftPoint, currentInterval.leftIndex, AngleDirection::kCounterclockwise);
              successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
              successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
              successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.leftInterval()->first, currentInterval.leftInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
              pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
              // Nothing else to do for this successor.
              return;
            }
          }
          // TODO: It could be the case that turning around this vertex might force us outside of our triangle.
          // What if after turning around this constraint, we hit the left of the interval on the way to the goal?
          //  Push a new interval to right of the interval, still for the current state. A follow-up iteration will check for that
          if (!currentInterval.rightInterval()) {
            throw std::runtime_error("Expecting right interval");
          }
          IntervalType successorInterval(currentState);
          successorInterval.setRoot(currentInterval.rightPoint, currentInterval.rightIndex, AngleDirection::kClockwise);
          successorInterval.setLeft(currentInterval.leftPoint, currentInterval.leftIndex);
          successorInterval.setRight(currentInterval.rightPoint, currentInterval.rightIndex);
          successorInterval.costToRoot = currentInterval.costToRoot + math::distance(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
          pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
          // Nothing else to do for this successor.
          return;
        } else if (currentState.hasEntryEdgeIndex()) {
          const auto [currentEdgeLeftPoint, currentEdgeLeftIndex, currentEdgeRightPoint, currentEdgeRightIndex] = getLeftAndRight(currentState);
          if (isAConstraintVertexForState(currentState, currentEdgeRightIndex)) {
            const auto [startOfRootToRightOfEntryEdge, endOfRootToRightOfEntryEdge] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, currentEdgeRightPoint, AngleDirection::kClockwise, agentRadius_);
            if (!math::lessThan(math::crossProductForSign(currentInterval.rightInterval()->first, currentInterval.rightInterval()->second, startOfRootToRightOfEntryEdge, endOfRootToRightOfEntryEdge), 0.0)) {
              // Check if the path to the goal intersects with the right vertex of this edge.
              const auto intersectionCount = math::lineSegmentIntersectsWithCircle(rootToGoalLineStart, goalPoint, currentEdgeRightPoint, agentRadius_);
              if (intersectionCount == 2) {
                VLOG(1) << "Ic=2";
                // The goal is on the other side of this entry edge's right vertex. We can turn around that.
                constraintToTurnAround.emplace(currentEdgeRightPoint, currentEdgeRightIndex, AngleDirection::kClockwise);
              } else if (intersectionCount == 1) {
                // This happens if the goal is inside the radius of the vertex.
                throw std::runtime_error("Not expecting 1 intersection to be possible. Case 1");
              }
            }
          }
        }
      }
      if (!constraintToTurnAround) {
        // Cannot be reached.
        VLOG(1) << "Cannot be reached.";
        // Nothing else to do for this successor.
        return;
      }
    }
  }

  if (!constraintToTurnAround) {
    VLOG(1) << "0 yuh";
    IntervalType successorInterval(successorState);
    successorInterval.setRoot(currentInterval.rootPoint, currentInterval.rootIndex, currentInterval.rootDirection);
    successorInterval.costToRoot = currentInterval.costToRoot;
    successorInterval.isGoal = true;
    pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
  } else {
    if (std::get<2>(*constraintToTurnAround) == AngleDirection::kNoDirection) {
      throw std::runtime_error("Angle direction is kNoDirection");
    }
    VLOG(1) << "Have a constraint to turn around";
    IntervalType successorInterval(successorState);
    successorInterval.setRoot(std::get<0>(*constraintToTurnAround), std::get<1>(*constraintToTurnAround), std::get<2>(*constraintToTurnAround));
    const auto [lineStart, lineEnd] = math::createCircleConsciousLine(currentInterval.rootPoint, currentInterval.rootDirection, std::get<0>(*constraintToTurnAround), std::get<2>(*constraintToTurnAround), agentRadius_);
    successorInterval.costToRoot = currentInterval.costToRoot + math::distance(lineStart, lineEnd); // TODO(cost): Add (estimate of) distance traveled around the currentInterval.root, if any.
    successorInterval.isGoal = true;
    pushSuccessor(&currentInterval, successorInterval, intervalHeap, visited, pushed, previous, goalPoint, debugAStarInfo);
  }
  // Goal successor is handled, the rest of this function is for non-goal successors.
  // Nothing else to do for this successor.
  return;
}

// Helper functions

template<typename NavmeshType>
std::tuple<Vector, typename Pathfinder<NavmeshType>::IndexType, Vector, typename Pathfinder<NavmeshType>::IndexType> Pathfinder<NavmeshType>::getLeftAndRight(const State &successorState) const {
  // Figure out which of the endpoints of this entry edge are "left" and which is "right" from the perspective of the state which enters this state. This is done by checking the direction from within the triangle and then flipping them to match the perspective of the entering state.
  const auto& [v1,v2,v3] = navmesh_.getTriangleVertices(successorState.getTriangleIndex());
  const Vector centerOfTriangle = (v1 + v2 + v3) / 3.0;
  auto [leftIndex, rightIndex] = navmesh_.getEdgeVertexIndices(successorState.getEntryEdgeIndex());
  auto leftVector = navmesh_.getVertex(leftIndex);
  auto rightVector = navmesh_.getVertex(rightIndex);
  if (math::crossProductForSign(centerOfTriangle, rightVector, centerOfTriangle, leftVector) > 0.0) {
    // Left and right are swapped.
    std::swap(leftIndex, rightIndex);
    std::swap(leftVector, rightVector);
  }
  return std::make_tuple(leftVector, leftIndex, rightVector, rightIndex);
}

} // namespace pathfinder

#endif // PATHFINDER_PATHFINDER_H_