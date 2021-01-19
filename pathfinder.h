#ifndef FILE_H_
#define FILE_H_

#include "triangle/triangle_api.h"

#include <QPointF>

#include <iostream>
#include <limits>
#include <map>
#include <vector>

struct State {
  int triangleNum{-1}, entryEdge{-1};
  bool isGoal{false};
};
bool operator==(const State &s1, const State &s2);
bool operator<(const State &s1, const State &s2);
std::ostream& operator<<(std::ostream& stream, const State &state);

class Pathfinder {
public:
  Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData);
  std::vector<int> findShortestPath(const QPointF &startPoint, const QPointF &goalPoint);
private:
  const double kCharacterRadius_{0.1};
  triangleio triangleData_, triangleVoronoiData_;

  int findTriangleForPoint(const QPointF &point) const;
  double distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point) const;
  double calculateHValue(const State &state, const QPointF &point) const;
  double calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, double> &gScores) const;
  std::vector<int> triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const;
  std::vector<State> getSuccessors(const State &state, int goalTriangle) const;
};

#endif // FILE_H_