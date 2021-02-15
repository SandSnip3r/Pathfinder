#ifndef FILE_H_
#define FILE_H_

#include "triangle/triangle_api.h"

#include <QPointF>

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
  QPointF startPoint, endPoint;
};

enum class AngleDirection {
  kPoint,
  kCounterclockwise,
  kClockwise
};

struct ArcPathSegment : public PathSegment {
  ArcPathSegment(const QPointF &center, const double radius, const AngleDirection direction) : circleCenter(center), circleRadius(radius), angleDirection(direction) {}
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
  triangleio triangleData_, triangleVoronoiData_;

  int findTriangleForPoint(const QPointF &point) const;
  QPointF midpointOfEdge(int edgeNum) const;
  double lengthOfEdge(int edgeNum) const;
  double distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point, QPointF *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const int edge1, const int edge2) const;
  double calculateHValue(const State &state, const QPointF &goalPoint) const;
  double calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, double> &gScores, const std::map<State, State> &previous) const;
  PathfindingAStarInfo triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const;
  std::vector<State> getSuccessors(const State &state, int goalTriangle) const;

  std::vector<std::pair<QPointF,QPointF>> buildCorridor(const std::vector<int> &trianglesInCorridor) const;
  std::vector<std::unique_ptr<PathSegment>> funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, const std::optional<QPointF> &goalPoint={}) const;
};

#endif // FILE_H_