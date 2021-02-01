#ifndef FILE_H_
#define FILE_H_

#include "triangle/triangle_api.h"

#include <QPointF>

#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

struct State {
  int triangleNum{-1}, entryEdge{-1};
  bool isGoal{false};
};
bool operator==(const State &s1, const State &s2);
bool operator<(const State &s1, const State &s2);
std::ostream& operator<<(std::ostream& stream, const State &state);

struct PathfindingAStarInfo {
  std::vector<int> triangleCorridor;
  std::set<int> trianglesSearched;
  std::set<int> trianglesDiscovered;
};

struct PathfindingResult {
  std::vector<QPointF> shortestPath;
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
  double lengthOfEdge(int edgeNum) const;
  double distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point, QPointF *pointUsedForDistanceCalculation=nullptr) const;
  double calculateArcLength(const int edge1, const int edge2) const;
  double calculateHValue(const State &state, const QPointF &goalPoint) const;
  double calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, double> &gScores) const;
  PathfindingAStarInfo triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const;
  std::vector<State> getSuccessors(const State &state, int goalTriangle) const;

  std::vector<std::pair<QPointF,QPointF>> buildCorridor(const std::vector<int> &trianglesInCorridor) const;
  std::vector<QPointF> funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, const QPointF &goalPoint) const;
};

#endif // FILE_H_