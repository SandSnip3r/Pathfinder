#ifndef PATHFINDER_PATHFINDER_H_
#define PATHFINDER_PATHFINDER_H_

#include "binary_min_heap.h"
#include "funnel.h"
#include "math_helpers.h"
#include "vector.h"

#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pathfinder {

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path);


/* TODO: Outline NavmeshType requirements
 *
 *
 */

template<typename NavmeshType>
class Pathfinder {
public:
  using State = typename NavmeshType::State;
  using IndexType = typename NavmeshType::IndexType;

  struct PathfindingAStarInfo {
    // TODO: This is just a debug structure, remove at some point
    std::vector<IndexType> triangleCorridor;
    std::unordered_set<IndexType> trianglesSearched;
    std::unordered_set<IndexType> trianglesDiscovered;
  };

  struct PathfindingResult {
    std::vector<std::unique_ptr<PathSegment>> shortestPath;
    PathfindingAStarInfo aStarInfo;
    void clear();
  };

  Pathfinder(const NavmeshType &navmesh, const double agentRadius);

  template<typename PointType>
  std::pair<Vector, IndexType> getPointAwayFromConstraint(const PointType &point) const;

  template<typename PointType>
  PathfindingResult findShortestPath(const PointType &startPoint, const PointType &goalPoint) const;
private:
  const NavmeshType &navmesh_;
  const double agentRadius_;
  
  mutable std::unordered_map<State, LengthFunnel> lengthFunnelCache_; // TODO: Dont use mutable

  // Checks if the given point collides with any of the constraint vertices of the triangle
  // If the distance to a constraint vertex is exactly the agent radius, then it will not collide
  // TODO: Does not check if the given point collides with edges of the triangle
  std::optional<Vector> getCollidingConstraint(const Vector &point, const IndexType triangleIndex) const;

  void buildShortestPathWithinSingleTriangle(const IndexType triangleIndex, const Vector &startPoint, const Vector &goalPoint, std::vector<std::unique_ptr<PathSegment>> &shortestPath) const;

  double distanceBetweenEdgeAndPoint(IndexType edgeIndex, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const IndexType edge1Index, const IndexType edge2Index) const;
  double calculateHValue(const State &state, const Vector &goalPoint) const;
  double calculateEstimateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, double> &gScores) const;
  std::tuple<double, Vector, std::optional<LengthFunnel>> calculateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, State> &previous) const;
  PathfindingAStarInfo triangleAStar(const Vector &startPoint, const State &startState, const Vector &goalPoint, const State &goalState) const;

  bool pathCanExist(const State &startState, const State &goalState) const;
  std::vector<std::pair<Vector,Vector>> buildCorridor(const std::vector<IndexType> &trianglesInCorridor) const;

  static std::vector<IndexType> rebuildPath(State state, const std::unordered_map<State, State> &previous);
};

#include "pathfinder.inl"

} // namespace pathfinder

#endif // PATHFINDER_PATHFINDER_H_