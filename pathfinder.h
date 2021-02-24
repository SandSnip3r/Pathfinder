#ifndef FILE_H_
#define FILE_H_

#include "math_helpers.h"
#include "triangle/triangle_api.h"

#include <QPointF>

#include <array>
#include <iostream>
#include <limits>
#include <memory>
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

struct PathSegment {
  virtual ~PathSegment();
};

struct StraightPathSegment : public PathSegment {
  StraightPathSegment(const QPointF &start, const QPointF &end) : startPoint(start), endPoint(end) {}
  StraightPathSegment(const StraightPathSegment &other) : startPoint(other.startPoint), endPoint(other.endPoint) {}
  QPointF startPoint, endPoint;
};

struct ArcPathSegment : public PathSegment {
  ArcPathSegment(const QPointF &center, const double radius, const AngleDirection direction) : circleCenter(center), circleRadius(radius), angleDirection(direction) {}
  ArcPathSegment(const ArcPathSegment &other) : circleCenter(other.circleCenter), circleRadius(other.circleRadius), angleDirection(other.angleDirection), startAngle(other.startAngle), endAngle(other.endAngle) {}
  QPointF circleCenter;
  double circleRadius;
  AngleDirection angleDirection;
  double startAngle, endAngle;
};

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
  PathfindingResult findShortestPath(const QPointF &startPoint, const QPointF &goalPoint);
  void setCharacterRadius(double value);
  int pointToIndex(const QPointF &point) const;
private:
  double characterRadius_{0};
  // Triangle data
  triangleio triangleData_, triangleVoronoiData_;

  // Pre-computed data from triangle data
  std::vector<std::array<int,3>> triangleEdges_;

  int findTriangleForPoint(const QPointF &point) const;
  QPointF midpointOfEdge(int edgeNum) const;
  double lengthOfEdge(int edgeNum) const;
  double distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point, QPointF *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const int edge1, const int edge2) const;
  double calculateHValue(const State &state, const QPointF &goalPoint) const;
  std::pair<double, QPointF> calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, State> &previous) const;
  PathfindingAStarInfo triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const;
  std::vector<State> getSuccessors(const State &state, int goalTriangle) const;

  bool pathCanExist(int startTriangle, int goalTriangle);
  int getSharedEdge(const int triangle1Num, const int triangle2Num) const;
  std::vector<std::pair<QPointF,QPointF>> buildCorridor(const std::vector<int> &trianglesInCorridor) const;
  std::vector<std::unique_ptr<PathSegment>> funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, std::optional<QPointF> &goalPoint) const;
};

#endif // FILE_H_