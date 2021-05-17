#ifndef PATHFINDER_PATHFINDER_H_
#define PATHFINDER_PATHFINDER_H_

#include "funnel.h"
#include "math_helpers.h"
#include "vector.h"

#include "a_star_navmesh_interface.h"

#include <array>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace pathfinder {

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path);

struct PathfindingAStarInfo {
  std::vector<int> triangleCorridor;
  std::set<int> trianglesSearched;
  std::set<int> trianglesDiscovered;
};

struct PathfindingResult {
  std::vector<std::unique_ptr<PathSegment>> shortestPath;
  PathfindingAStarInfo aStarInfo;
  void clear();
};

class Pathfinder {
public:
  Pathfinder(const navmesh::AStarNavmeshInterface &navmesh);
  PathfindingResult findShortestPath(const Vector &startPoint, const Vector &goalPoint) const;
  void setCharacterRadius(double value);
private:
  double agentRadius_{0.0};
  const navmesh::AStarNavmeshInterface &navmesh_;
  
  // Pre-computed data from triangle data
  std::vector<std::array<int,3>> triangleEdges_;

  mutable std::map<navmesh::State, LengthFunnel> lengthFunnelCache_; // TODO: Dont use mutable

  // Checks if the given point collides with any of the constraint vertices of the triangle
  // If the distance to a constraint vertex is exactly the agent radius, then it will not collide
  // TODO: Does not check if the given point collides with edges of the triangle
  bool collidesWithConstraint(const Vector &point, const int triangleIndex) const;

  void buildShortestPathWithinSingleTriangle(const int triangleIndex, const Vector &startPoint, const Vector &goalPoint, std::vector<std::unique_ptr<PathSegment>> &shortestPath) const;

  double distanceBetweenEdgeAndPoint(int edgeNum, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const int edge1, const int edge2) const;
  double calculateHValue(const navmesh::State &state, const Vector &goalPoint) const;
  double calculateEstimateGValue(const navmesh::State &state, const navmesh::State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<navmesh::State, double> &gScores) const;
  std::tuple<double, Vector, std::optional<LengthFunnel>> calculateGValue(const navmesh::State &state, const navmesh::State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<navmesh::State, navmesh::State> &previous) const;
  PathfindingAStarInfo triangleAStar(const Vector &startPoint, int startTriangle, const Vector &goalPoint, int goalTriangle) const;

  bool pathCanExist(int startTriangle, int goalTriangle) const;
  std::vector<std::pair<Vector,Vector>> buildCorridor(const std::vector<int> &trianglesInCorridor) const;
};

} // namespace pathfinder

#endif // PATHFINDER_PATHFINDER_H_