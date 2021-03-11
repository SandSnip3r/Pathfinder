#ifndef FILE_H_
#define FILE_H_

#include "funnel.h"
#include "math_helpers.h"
#include "vector.h"
#include "triangle/triangle_api.h"

#include <array>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

struct State {
  int triangleNum{-1}, entryEdge{-1};
  bool isGoal{false};
};
bool operator==(const State &s1, const State &s2);
bool operator<(const State &s1, const State &s2);
std::ostream& operator<<(std::ostream& stream, const State &state);

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
  Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData);
  PathfindingResult findShortestPath(const Vector &startPoint, const Vector &goalPoint);
  void setCharacterRadius(double value);
  int pointToIndex(const Vector &point) const;
private:
  double characterRadius_{0};
  // Triangle data
  triangleio triangleData_, triangleVoronoiData_;
  
  // Pre-computed data from triangle data
  std::vector<std::array<int,3>> triangleEdges_;

  mutable std::map<State, LengthFunnel> lengthFunnelCache_; // TODO: Dont use mutable

  bool pointIsInTriangle(const Vector &point, const int triangleNum) const;
  int findTriangleForPoint(const Vector &point) const;
  Vector midpointOfEdge(int edgeNum) const;
  double lengthOfEdge(int edgeNum) const;
  std::pair<Vector,Vector> getEdge(int edgeNum) const;
  double distanceBetweenEdgeAndPoint(int edgeNum, const Vector &point, Vector *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const int edge1, const int edge2) const;
  double calculateHValue(const State &state, const Vector &goalPoint) const;
  double calculateEstimateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<State, double> &gScores) const;
  std::tuple<double, Vector, std::optional<LengthFunnel>> calculateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<State, State> &previous) const;
  PathfindingAStarInfo triangleAStar(const Vector &startPoint, int startTriangle, const Vector &goalPoint, int goalTriangle) const;
  std::vector<State> getSuccessors(const State &state, int goalTriangle) const;

  bool pathCanExist(int startTriangle, int goalTriangle);
  int getSharedEdge(const int triangle1Num, const int triangle2Num) const;
  std::vector<std::pair<Vector,Vector>> buildCorridor(const std::vector<int> &trianglesInCorridor) const;
};

#endif // FILE_H_