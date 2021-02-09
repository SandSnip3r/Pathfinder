#include "math_helpers.h"
#include "pathfinder.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <string>

struct Apex {
  QPointF apexPoint;
  AngleDirection apexType;
};

std::vector<int> rebuildPath(State state, const std::map<State, State> &previous);
void addToLeftOfFunnel(std::deque<QPointF> &funnel, Apex &apex, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
void addToRightOfFunnel(std::deque<QPointF> &funnel, Apex &apex, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString, const bool isGoal = false);
void finishFunnel(std::deque<QPointF> &funnel, Apex &apex, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
bool counterclockwiseTo(const double theta, const double phi);
bool clockwiseTo(const double theta, const double phi);
double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
double angle(const QPointF &point1, const AngleDirection point1Direction, const QPointF &point2, const AngleDirection point2Direction, const double circleRadius);
std::pair<QPointF, QPointF> intersectionsPointsOfTangentLinesToCircle(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
void addSegmentToPath(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
std::pair<QPointF, QPointF> createCircleConciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius);
double distanceBetweenEdgeAndPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &point, QPointF *pointUsedForDistanceCalculation=nullptr);

PathSegment::~PathSegment() {}

Pathfinder::Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData) : triangleData_(triangleData), triangleVoronoiData_(triangleVoronoiData) {}

PathfindingResult Pathfinder::findShortestPath(const QPointF &startPoint, const QPointF &goalPoint) {
  int startTriangle = findTriangleForPoint(startPoint);
  int goalTriangle = findTriangleForPoint(goalPoint);
  PathfindingResult result;
  if (startTriangle == goalTriangle) {
    // Only one triangle involved
    // Shortest path is a straight line between start and goal
    result.shortestPath.emplace_back(std::unique_ptr<StraightPathSegment>(new StraightPathSegment(startPoint, goalPoint)));
    // Only one triangle in corridor
    result.aStarInfo.triangleCorridor.emplace_back(startTriangle);
    // Only one triangle looked at
    result.aStarInfo.trianglesDiscovered.emplace(startTriangle);
    // Only one triangle evaluated completely
    result.aStarInfo.trianglesSearched.emplace(startTriangle);
    return result;
  }
  result.aStarInfo = triangleAStar(startPoint, startTriangle, goalPoint, goalTriangle);
  if (!result.aStarInfo.triangleCorridor.empty()) {
    // A path was found
    result.shortestPath = funnel(result.aStarInfo.triangleCorridor, startPoint, goalPoint);
  }
  return result;
}

std::vector<std::pair<QPointF,QPointF>> Pathfinder::buildCorridor(const std::vector<int> &trianglesInCorridor) const {
  // TODO: Maybe start this entire pathfinding process with these checks so that we know we're safe and dont need to check them all the time
  if (triangleData_.trianglelist == nullptr) {
    throw std::runtime_error("Cannot build corridor when trianglelist is null");
  }
  if (triangleData_.pointlist == nullptr) {
    throw std::runtime_error("Cannot build corridor when pointlist is null");
  }
  if (triangleData_.edgelist == nullptr) {
    throw std::runtime_error("Cannot build corridor when edgelist is null");
  }
  if (triangleVoronoiData_.edgelist == nullptr) {
    throw std::runtime_error("Cannot build corridor when voronoi edgelist is null");
  }

  std::vector<std::pair<QPointF,QPointF>> corridorSegments;
  for (int i=1; i<trianglesInCorridor.size(); ++i) {
    // find common edge between triangle i,i-1
    const int triangleANum = trianglesInCorridor.at(i-1);
    const int triangleBNum = trianglesInCorridor.at(i);
    if (triangleANum >= triangleData_.numberoftriangles || triangleBNum >= triangleData_.numberoftriangles) {
      throw std::runtime_error("Invalid triangle number while building corridor");
    }
    int sharedEdge = -1;
    for (int edgeNum=0; edgeNum<triangleVoronoiData_.numberofedges; ++edgeNum) {
      const int triangle1Number = triangleVoronoiData_.edgelist[edgeNum*2];
      const int triangle2Number = triangleVoronoiData_.edgelist[edgeNum*2+1];
      if ((triangle1Number == triangleANum &&
          triangle2Number == triangleBNum) ||
          (triangle1Number == triangleBNum &&
          triangle2Number == triangleANum)) {
        // Found the shared edge
        sharedEdge = edgeNum;
        break;
      }
    }
    if (sharedEdge == -1) {
      throw std::runtime_error("Unabled to find shared edge between two triangles");
    }
    if (sharedEdge >= triangleData_.numberofedges) {
      throw std::runtime_error("Shared edge is not a valid edge");
    }
    // Get two points of edge
    const auto sharedEdgeVertexAIndex = triangleData_.edgelist[sharedEdge*2];
    const auto sharedEdgeVertexBIndex = triangleData_.edgelist[sharedEdge*2+1];
    if (sharedEdgeVertexAIndex >= triangleData_.numberofpoints || sharedEdgeVertexBIndex >= triangleData_.numberofpoints) {
      throw std::runtime_error("Shared edge references points which do not exist");
    }
    const QPointF sharedEdgeVertexA{triangleData_.pointlist[sharedEdgeVertexAIndex*2], triangleData_.pointlist[sharedEdgeVertexAIndex*2+1]};
    const QPointF sharedEdgeVertexB{triangleData_.pointlist[sharedEdgeVertexBIndex*2], triangleData_.pointlist[sharedEdgeVertexBIndex*2+1]};
    corridorSegments.emplace_back(sharedEdgeVertexA, sharedEdgeVertexB);
  }
  return corridorSegments;
}

int Pathfinder::pointToIndex(const QPointF &point) const {
  // TODO: Remove once done debugging
  for (int i=0; i<triangleData_.numberofpoints; ++i) {
    if (point.x() == triangleData_.pointlist[i*2] && point.y() == triangleData_.pointlist[i*2+1]) {
      return i;
    }
  }
  throw std::runtime_error("Unknown point");
}

std::vector<std::unique_ptr<PathSegment>> Pathfinder::funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, const QPointF &goalPoint) const {
  std::function<std::string(const QPointF&)> pointToStringFunc = [this, &startPoint, &goalPoint](const QPointF &point) -> std::string {
    if (point == startPoint) {
      return "[START]";
    } else if (point == goalPoint) {
      return "[GOAL]";
    } else {
      try {
        return std::to_string(pointToIndex(point));
      } catch(...) {
        std::string result = "(";
        result += std::to_string(point.x());
        result += ",";
        result += std::to_string(point.y());
        result += ")";
        return result;
      }
    }
  };
  auto apexToString = [&pointToStringFunc](const Apex &apex) {
    std::string result = "(" + pointToStringFunc(apex.apexPoint);
    if (apex.apexType == AngleDirection::kClockwise) {
      result += ",cw";
    } else if (apex.apexType == AngleDirection::kCounterclockwise) {
      result += ",ccw";
    }
    result += ")";
    return result;
  };
  auto printFunnelAndApex = [&pointToStringFunc, &apexToString](const std::deque<QPointF> &funnel, const Apex &apex) {
    std::cout << "[";
    for (const auto &i : funnel) {
      std::cout << pointToStringFunc(i) << ',';
    }
    std::cout << "], apex = " << apexToString(apex) << std::endl;
  };

  if (trianglesInCorridor.empty()) {
    throw std::runtime_error("Trying to funnel for an empty corridor");
  }
  if (trianglesInCorridor.size() == 1) {
    // Start and goal are in the same triangle, the path is a straight line from start to goal
    std::vector<std::unique_ptr<PathSegment>> path;
    path.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(startPoint, goalPoint)));
    std::cout << "Single segment!" << std::endl;
    return path;
  }
  // Build a list of "edges" that we'll need to pass through in this corridor
  using CorridorType = std::vector<std::pair<QPointF,QPointF>>;
  using PathType = std::vector<std::unique_ptr<PathSegment>>;
  CorridorType corridor = buildCorridor(trianglesInCorridor);
  PathType path;
  Apex currentApex;

  std::deque<QPointF> funnel;
  // Initialize funnel with start point
  funnel.push_front(startPoint);
  // Initialize apex as start point
  currentApex.apexPoint = startPoint;
  currentApex.apexType = AngleDirection::kPoint;

  // Start by adding
  QPointF left, right;
  // First and second point are the two edges of the first edge
  std::tie(left, right) = corridor.at(0);
  const bool areCorrectOrder = (math::crossProduct(startPoint, right, startPoint, left) > 0);
  if (!areCorrectOrder) {
    // incorrect order (left is actually right), swap
    std::swap(left, right);
  }
  std::cout << "=============================================" << std::endl;
  std::cout << "Starting funnel: ";
  printFunnelAndApex(funnel, currentApex);

  addToLeftOfFunnel(funnel, currentApex, left, path, characterRadius_, pointToStringFunc);
  std::cout << "Funnel after adding left: ";
  printFunnelAndApex(funnel, currentApex);

  addToRightOfFunnel(funnel, currentApex, right, path, characterRadius_, pointToStringFunc);
  std::cout << "Funnel before loop (after adding right): ";
  printFunnelAndApex(funnel, currentApex);

  for (int i=1; i<corridor.size(); ++i) {
    QPointF newPoint1, newPoint2;
    std::tie(newPoint1, newPoint2) = corridor.at(i);
    if (newPoint1 == right) {
      // newPoint2 will be left
      left = newPoint2;
      addToLeftOfFunnel(funnel, currentApex, left, path, characterRadius_, pointToStringFunc);
    } else if (newPoint1 == left) {
      // newPoint2 will be right
      right = newPoint2;
      addToRightOfFunnel(funnel, currentApex, right, path, characterRadius_, pointToStringFunc);
    } else if (newPoint2 == right) {
      // newPoint1 will be left
      left = newPoint1;
      addToLeftOfFunnel(funnel, currentApex, left, path, characterRadius_, pointToStringFunc);
    } else {
      // newPoint1 will be right
      right = newPoint1;
      addToRightOfFunnel(funnel, currentApex, right, path, characterRadius_, pointToStringFunc);
    }
    std::cout << "Funnel : ";
    printFunnelAndApex(funnel, currentApex);
  }

  // Finally, add the goal to the right of the funnel
  addToRightOfFunnel(funnel, currentApex, goalPoint, path, characterRadius_, pointToStringFunc, true);
  std::cout << "Funnel after loop : ";
  printFunnelAndApex(funnel, currentApex);
  // And finish the algorithm, closing out the funnel
  finishFunnel(funnel, currentApex, path, characterRadius_, pointToStringFunc);
  std::cout << "Multiple segments! (" << path.size() << ")" << std::endl;
  return path;
}

void Pathfinder::setCharacterRadius(double value) {
  characterRadius_ = value;
}

int Pathfinder::findTriangleForPoint(const QPointF &point) const {
  if (triangleData_.trianglelist == nullptr) {
    throw std::runtime_error("Triangle list is null!");
  }
  if (triangleData_.pointlist == nullptr) {
    throw std::runtime_error("Point list is null!");
  }
  // TODO: Check for more nulls in lists

  // Loop over all triangles
  for (int triangleNumber=0; triangleNumber<triangleData_.numberoftriangles; ++triangleNumber) {
    const int vertexIndexA = triangleData_.trianglelist[triangleNumber*3];
    const int vertexIndexB = triangleData_.trianglelist[triangleNumber*3+1];
    const int vertexIndexC = triangleData_.trianglelist[triangleNumber*3+2];
    if (vertexIndexA >= triangleData_.numberofpoints ||
        vertexIndexB >= triangleData_.numberofpoints ||
        vertexIndexC >= triangleData_.numberofpoints) {
      // TODO: Maybe it's excessive to throw? Maybe just skip this triangle
      throw std::runtime_error("Triangle references vertex which does not exist");
    }
    const QPointF vertexA{triangleData_.pointlist[vertexIndexA*2], triangleData_.pointlist[vertexIndexA*2+1]};
    const QPointF vertexB{triangleData_.pointlist[vertexIndexB*2], triangleData_.pointlist[vertexIndexB*2+1]};
    const QPointF vertexC{triangleData_.pointlist[vertexIndexC*2], triangleData_.pointlist[vertexIndexC*2+1]};

    // Triangles' vertices are listed in CCW order (might matter for checking if a point lies within a triangle)
    if (math::isPointInTriangle(point, vertexA, vertexB, vertexC)) {
      return triangleNumber;
    }
  }
  throw std::runtime_error("Unable to find triangle for point");
}

double Pathfinder::calculateArcLength(const int edge1, const int edge2) const {
  // Return the length of the arc from one edge to another (they share a vertex)
  if (edge1 < 0 || edge2 < 0) {
    throw std::runtime_error("Invalid edge given");
  }
  if (edge1 >= triangleData_.numberofedges || edge2 >= triangleData_.numberofedges) {
    throw std::runtime_error("Edge number is not within the triangle data");
  }
  const auto edge1VertexAIndex = triangleData_.edgelist[edge1*2];
  const auto edge1VertexBIndex = triangleData_.edgelist[edge1*2+1];
  if (edge1VertexAIndex >= triangleData_.numberofpoints || edge1VertexBIndex >= triangleData_.numberofpoints) {
    throw std::runtime_error("Edge 1 references points which do not exist");
  }
  const auto edge2VertexAIndex = triangleData_.edgelist[edge2*2];
  const auto edge2VertexBIndex = triangleData_.edgelist[edge2*2+1];
  if (edge2VertexAIndex >= triangleData_.numberofpoints || edge2VertexBIndex >= triangleData_.numberofpoints) {
    throw std::runtime_error("Edge 2 references points which do not exist");
  }
  
  const QPointF edge1VertexA{triangleData_.pointlist[edge1VertexAIndex*2], triangleData_.pointlist[edge1VertexAIndex*2+1]};
  const QPointF edge1VertexB{triangleData_.pointlist[edge1VertexBIndex*2], triangleData_.pointlist[edge1VertexBIndex*2+1]};
  const QPointF edge2VertexA{triangleData_.pointlist[edge2VertexAIndex*2], triangleData_.pointlist[edge2VertexAIndex*2+1]};
  const QPointF edge2VertexB{triangleData_.pointlist[edge2VertexBIndex*2], triangleData_.pointlist[edge2VertexBIndex*2+1]};
  const double angle = math::angleBetweenVectors(edge1VertexA, edge1VertexB, edge2VertexA, edge2VertexB);
  // TODO: Sometimes this arclength causes weird paths
  return 0;
  // return 0.1 * characterRadius_ * angle;
  // return characterRadius_ * angle; // Original from paper
}

double Pathfinder::calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, double> &gScores) const {
  // The common edge between triangles `state` and `parentState`
  const int commonEdge = state.entryEdge;
  const double parentGValue = gScores.at(parentState);
  const double hValue = calculateHValue(state, goalPoint);

  // Max of {
  // 1. Distance between start and closest point on edge `commonEdge`
  double val1;
  if (state.isGoal) {
    // Straight line from start to goal
    val1 = math::distance(startPoint, goalPoint);
  } else {
    // Straight line from start and closest point of edge to start
    val1 = distanceBetweenEdgeAndPoint(commonEdge, startPoint);
  }
  
  // 2. parentState.gValue + (arc around vertex shared by parentState.entryEdge and `commonEdge`)
  double val2 = parentGValue;
  if (state.isGoal) {
    // Need to add distance from entry edge to goal point
    val2 += distanceBetweenEdgeAndPoint(state.entryEdge, goalPoint);
  } else if (parentState.entryEdge != -1) {
    // Can calculate arc-length
    val2 += calculateArcLength(parentState.entryEdge, commonEdge);
  }
  
  // 3. parentState.gValue + (parentState.hValue - state.hValue)
  double parentHeuristicValue;
  if (parentState.entryEdge == -1) {
    // This is the start
    parentHeuristicValue = math::distance(startPoint, goalPoint);
  } else {
    parentHeuristicValue = calculateHValue(parentState, goalPoint);
  }
  const double val3 = parentGValue + (parentHeuristicValue - hValue);
  // }
  // std::cout << "  Potential g values for state " << state; //DEBUGPRINTS
  // printf(" are [%.9f,%.9f,%.9f]\n",val1,val2,val3); //DEBUGPRINTS
  // std::cout << std::flush; //DEBUGPRINTS
  return std::max({val1, val2, val3});
}

double Pathfinder::lengthOfEdge(int edgeNum) const {
  if (edgeNum < 0) {
    throw std::runtime_error("Invalid edge");
  }
  if (edgeNum >= triangleData_.numberofedges) {
    throw std::runtime_error("Edge number is not within the triangle data");
  }
  const auto vertexAIndex = triangleData_.edgelist[edgeNum*2];
  const auto vertexBIndex = triangleData_.edgelist[edgeNum*2+1];
  if (vertexAIndex >= triangleData_.numberofpoints || vertexBIndex >= triangleData_.numberofpoints) {
    throw std::runtime_error("Edge references points which do not exist");
  }
  const QPointF vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  const QPointF vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};
  return math::distance(vertexA, vertexB);
}

double Pathfinder::distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point, QPointF *pointUsedForDistanceCalculation) const {
  // Return the distance between the closest end of the given edge and the given point
  if (edgeNum < 0) {
    throw std::runtime_error("Invalid edge");
  }
  if (edgeNum >= triangleData_.numberofedges) {
    throw std::runtime_error("Edge number is not within the triangle data");
  }
  const auto vertexAIndex = triangleData_.edgelist[edgeNum*2];
  const auto vertexBIndex = triangleData_.edgelist[edgeNum*2+1];
  if (vertexAIndex >= triangleData_.numberofpoints || vertexBIndex >= triangleData_.numberofpoints) {
    throw std::runtime_error("Edge references points which do not exist");
  }
  const QPointF vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  const QPointF vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};

  return ::distanceBetweenEdgeAndPoint(vertexA, vertexB, point, pointUsedForDistanceCalculation);
}

double Pathfinder::calculateHValue(const State &state, const QPointF &goalPoint) const {
  // The h-value is the Euclidean distance between the goalPoint and the closest point to it on this edge (entry edge)
  if (state.entryEdge == -1) {
    throw std::runtime_error("Logic error, calculating h value for state which has no entry edge");
  }
  if (state.isGoal) {
    // Heuristic from the goal to the goal is 0
    return 0;
  }
  return distanceBetweenEdgeAndPoint(state.entryEdge, goalPoint);
}

std::vector<State> Pathfinder::getSuccessors(const State &state, int goalTriangle) const {
  if (state.isGoal) {
    throw std::runtime_error("Trying to get successors of goal");
  }

  if (state.triangleNum == goalTriangle) {
    // This is the goal triangle, only successor is the goal point itself
    State goalState{state};
    goalState.isGoal = true;
    return {goalState};
  }

  if (state.triangleNum >= triangleData_.numberoftriangles) {
    throw std::runtime_error("Triangle is not in data");
  }

  std::vector<State> result;
  constexpr const bool kAvoidObstacles = true;
  constexpr const bool kCareAboutCharacterRadius = true;
  auto characterFitsThroughTriangle = [this, &kCareAboutCharacterRadius](const int entryEdge, const int exitEdge) -> bool {
    // TODO: Need to improve
    //  Some of these edges might not be constrained
    //  See C:\Users\Victor\Documents\ShareX\Screenshots\2021-02\drawPolygon_lYhSzArwEG.png
    //  Every point is on a constraint, but not necessarily every edge is
    if (!kCareAboutCharacterRadius || characterRadius_ == 0.0) {
      return true;
    }
    if (entryEdge < 0 || exitEdge < 0) {
      throw std::runtime_error("Invalid edge given");
    }
    if (entryEdge >= triangleData_.numberofedges || exitEdge >= triangleData_.numberofedges) {
      throw std::runtime_error("Edge number is not within the triangle data");
    }
    const auto entryEdgeVertexAIndex = triangleData_.edgelist[entryEdge*2];
    const auto entryEdgeVertexBIndex = triangleData_.edgelist[entryEdge*2+1];
    if (entryEdgeVertexAIndex >= triangleData_.numberofpoints || entryEdgeVertexBIndex >= triangleData_.numberofpoints) {
      throw std::runtime_error("Entry edge references points which do not exist");
    }
    const auto exitEdgeVertexAIndex = triangleData_.edgelist[exitEdge*2];
    const auto exitEdgeVertexBIndex = triangleData_.edgelist[exitEdge*2+1];
    if (exitEdgeVertexAIndex >= triangleData_.numberofpoints || exitEdgeVertexBIndex >= triangleData_.numberofpoints) {
      throw std::runtime_error("Exit edge references points which do not exist");
    }
    
    const QPointF entryEdgeVertexA{triangleData_.pointlist[entryEdgeVertexAIndex*2], triangleData_.pointlist[entryEdgeVertexAIndex*2+1]};
    const QPointF entryEdgeVertexB{triangleData_.pointlist[entryEdgeVertexBIndex*2], triangleData_.pointlist[entryEdgeVertexBIndex*2+1]};
    const QPointF exitEdgeVertexA{triangleData_.pointlist[exitEdgeVertexAIndex*2], triangleData_.pointlist[exitEdgeVertexAIndex*2+1]};
    const QPointF exitEdgeVertexB{triangleData_.pointlist[exitEdgeVertexBIndex*2], triangleData_.pointlist[exitEdgeVertexBIndex*2+1]};

    return ((math::distance(entryEdgeVertexA, entryEdgeVertexB) >= (characterRadius_*2)) && math::distance(exitEdgeVertexA, exitEdgeVertexB) >= (characterRadius_*2));
    // TODO: This is a better approximation until the below code handles unconstrained edges

    QPointF sharedPoint, oppositeEdgeStart, oppositeEdgeEnd;

    if (entryEdgeVertexAIndex == exitEdgeVertexAIndex) {
      sharedPoint = entryEdgeVertexA;
      oppositeEdgeStart = entryEdgeVertexB;
      oppositeEdgeEnd = exitEdgeVertexB;
    } else if (entryEdgeVertexAIndex == exitEdgeVertexBIndex) {
      sharedPoint = entryEdgeVertexA;
      oppositeEdgeStart = entryEdgeVertexB;
      oppositeEdgeEnd = exitEdgeVertexA;
    } else if (entryEdgeVertexBIndex == exitEdgeVertexAIndex) {
      sharedPoint = entryEdgeVertexB;
      oppositeEdgeStart = entryEdgeVertexA;
      oppositeEdgeEnd = exitEdgeVertexB;
    } else if (entryEdgeVertexBIndex == exitEdgeVertexBIndex) {
      sharedPoint = entryEdgeVertexB;
      oppositeEdgeStart = entryEdgeVertexA;
      oppositeEdgeEnd = exitEdgeVertexA;
    } else {
      throw std::runtime_error("No shared point between these two edges");
    }

    return (::distanceBetweenEdgeAndPoint(oppositeEdgeStart, oppositeEdgeEnd, sharedPoint) >= (characterRadius_*2));
  };
  
  // For each neighboring triangle index
  for (int neighborNum=state.triangleNum*3; neighborNum<state.triangleNum*3+3; ++neighborNum) {
    const auto neighborTriangleNum = triangleData_.neighborlist[neighborNum];
    if (neighborTriangleNum != -1) {
      // Neighbor exists
      // Find out which edge is shared between this neighbor and the original triangle
      // TODO: Optimize, probably shouldnt need to loop through all edges
      //  Might be worth building a neighbor list
      for (int edgeNum=0; edgeNum<triangleVoronoiData_.numberofedges; ++edgeNum) {
        const int triangle1Number = triangleVoronoiData_.edgelist[edgeNum*2];
        const int triangle2Number = triangleVoronoiData_.edgelist[edgeNum*2+1];
        if (!(kAvoidObstacles && triangleData_.edgemarkerlist[edgeNum] != 0)) {
          // Non-constraint edge
          if ((triangle1Number == state.triangleNum &&
              triangle2Number == neighborTriangleNum) ||
              (triangle1Number == neighborTriangleNum &&
              triangle2Number == state.triangleNum)) {
            // Found the shared edge
            if (edgeNum != state.entryEdge) {
              // Only add a successor if it isnt the previous triangle
              if (state.entryEdge < 0 || characterFitsThroughTriangle(state.entryEdge, edgeNum)) {
                State successor;
                successor.entryEdge = edgeNum;
                successor.triangleNum = neighborTriangleNum;
                result.push_back(successor);
                break;
              }
            }
          }
        }
      }
    }
  }
  return result;
}

PathfindingAStarInfo Pathfinder::triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const {
  PathfindingAStarInfo result;
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Trying to find shortest path from triangle " << startTriangle << " to triangle " << goalTriangle << std::endl; //DEBUGPRINTS
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  std::vector<State> openSet; // "open set"

  std::map<State, State> previous;

  // Storing the best g score for each state
  std::map<State, double> gScores;
  // Storing the best f score for each state
  std::map<State, double> fScores;

  // Create the start state
  State startState;
  startState.entryEdge = -1; // There is no entry edge for the start state
  startState.triangleNum = startTriangle;
  openSet.push_back(startState);

  // Initialize the result as having touched the start state
  result.trianglesDiscovered.emplace(startState.triangleNum);

  // Set g score for start state
  gScores.emplace(startState, 0);
  // Calculate heuristic value for start point
  const auto distanceFromStartToGoal = math::distance(startPoint, goalPoint);
  // std::cout << "Distance from start to goal: " << distanceFromStartToGoal << std::endl;
  // Set f score for start state
  fScores.emplace(startState, distanceFromStartToGoal);

  while (!openSet.empty()) {
    // std::cout << "Evaluating possible next options ["; //DEBUGPRINTS
    // for (const State &s : openSet) { //DEBUGPRINTS
      // std::cout << '{' << s << ',' << fScores.at(s) << "},"; //DEBUGPRINTS
    // } //DEBUGPRINTS
    // std::cout << ']' << std::endl; //DEBUGPRINTS

    // Find the state with minimum fScore
    auto minElementIt = std::min_element(openSet.begin(), openSet.end(), [&fScores](const State &s1, const State &s2) {
      // We may assume that every state is in the fScore map already
      return math::lessThan(fScores.at(s1), fScores.at(s2));
    });
    if (minElementIt == openSet.end()) {
      throw std::runtime_error("No minimum element in a nonempty vector. Logic error");
    }
    
    // Copy state, since we will delete it from the open set
    const State currentState = *minElementIt;
    
    if (currentState.isGoal) {
      // Done!
      // std::cout << "Best option is the goal, we are done" << std::endl;
      result.triangleCorridor = rebuildPath(currentState, previous);
      std::cout << std::endl; // TODO: Remove
      return result;
    }

    // std::cout << "We chose " << currentState << " as our next candidate" << std::endl;
    openSet.erase(minElementIt);

    // Look at successors
    auto successors = getSuccessors(currentState, goalTriangle);
    for (const auto &successor : successors) {
      // Touched this successor
      result.trianglesDiscovered.emplace(successor.triangleNum);

      // std::cout << "  Evaluatating successor " << successor << std::endl;
      if (gScores.find(successor) == gScores.end()) {
        // No gScore yet for this state, initialize
        gScores.emplace(successor, std::numeric_limits<double>::max());
      }
      if (fScores.find(successor) == fScores.end()) {
        // No fScore yet for this state, initialize
        fScores.emplace(successor, std::numeric_limits<double>::max());
      }
      
      const double gValueOfSuccessor = calculateGValue(successor, currentState, startPoint, goalPoint, gScores);
      if (math::lessThan(gValueOfSuccessor, gScores.at(successor))) {
        // TODO: This comparison was suffering from a floating point issue
        // This path is better than any previous one
        // if (gScores.at(successor) != std::numeric_limits<double>::max()) { //DEBUGPRINTS
          // printf("  better gScore found %.12f vs %.12f\n", gScores.at(successor), gValueOfSuccessor); //DEBUGPRINTS
          // printf("    difference: %E\n", gValueOfSuccessor-gScores.at(successor)); //DEBUGPRINTS
          // std::cout << std::flush; //DEBUGPRINTS
        // } //DEBUGPRINTS
        gScores.at(successor) = gValueOfSuccessor;

        // Update previous
        previous[successor] = currentState;

        // Update fScore
        const double hValueOfSuccessor = calculateHValue(successor, goalPoint);
        const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
        // printf("    Resulting heuristic: %.9f, and fScore: %.9f\n", hValueOfSuccessor, fValueOfSuccessor); //DEBUGPRINTS
        // std::cout << std::flush; //DEBUGPRINTS
        fScores.at(successor) = fValueOfSuccessor;

        // Add to open set
        if (std::find(openSet.begin(), openSet.end(), successor) == openSet.end()) {
          openSet.push_back(successor);
        }
      }
    }
    // std::cout << std::endl;

    // Completely evaluated this state
    result.trianglesSearched.emplace(currentState.triangleNum);
  }

  // Open set is empty, but no path was found
  std::cout << std::endl; // TODO: Remove
  return result;
}

/* ========================================================================== **
** ============================= Free functions ============================= **
** ========================================================================== */

std::vector<int> rebuildPath(State state, const std::map<State, State> &previous) {
  std::vector<int> result;
  while (previous.find(state) != previous.end()) {
    if (!state.isGoal) {
      result.emplace_back(state.triangleNum);
    }
    state = previous.at(state);
  }
  result.emplace_back(state.triangleNum);
  std::reverse(result.begin(), result.end());
  return result;
}

bool counterclockwiseTo(const double theta, const double phi) {
  // Checking if rotating from theta to phi is counterclockwise
  return !clockwiseTo(theta, phi);
}

bool clockwiseTo(const double theta, const double phi) {
  const double kPi = 3.14159;
  // Checking if rotating from theta to phi is clockwise
  // Move theta to 0 and move phi by same angle (-theta)
  double newPhi = phi - theta;
  if (newPhi < 0) {
    newPhi += 2*kPi;
  }
  return (newPhi >= kPi);
}

double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius) {
  // Find the two lines that are tangent to the circle and intersect with the given point
  double distanceToCircle = math::distance(point,centerOfCircle);
  double lengthOfTangentLine = sqrt(distanceToCircle*distanceToCircle-circleRadius*circleRadius);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);
  return angleOfTangentLine;
}

std::pair<QPointF, QPointF> intersectionsPointsOfTangentLinesToCircle(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius) {
  // Find the two lines that are tangent to the circle and intersect with the given point
  double distanceToCircle = math::distance(point,centerOfCircle);
  double lengthOfTangentLine = sqrt(distanceToCircle*distanceToCircle-circleRadius*circleRadius);
  double angleOfTangentLine = asin(circleRadius/distanceToCircle);

  // Calculate angle for center of circle
  double angleOfCenterOfCircle = math::angle(point, centerOfCircle);

  // First point is counterclockwise to circle
  // Second point is clockwise to circle
  double x1 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle+angleOfTangentLine);
  double y1 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle+angleOfTangentLine);

  double x2 = point.x() + lengthOfTangentLine * cos(angleOfCenterOfCircle-angleOfTangentLine);
  double y2 = point.y() + lengthOfTangentLine * sin(angleOfCenterOfCircle-angleOfTangentLine);
  return {QPointF(x1, y1), QPointF(x2, y2)};
}

double angle(const QPointF &point1, const AngleDirection point1Direction, const QPointF &point2, const AngleDirection point2Direction, const double circleRadius) {
  double angleBetweenPoints = math::angle(point1, point2);
  
  if (point1Direction == AngleDirection::kPoint && point2Direction != AngleDirection::kPoint) {
    // Point to circle
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(point1, point2, circleRadius);
    if (point2Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the left point
      angleBetweenPoints += angleToTangent;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the right point
      angleBetweenPoints -= angleToTangent;
    }
  } else if (point1Direction != AngleDirection::kPoint && point2Direction == AngleDirection::kPoint) {
    // Circle to point
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(point2, point1, circleRadius);
    if (point1Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the right point (from the perspective of point2)
      angleBetweenPoints -= angleToTangent;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the left point (from the perspective of point2)
      angleBetweenPoints += angleToTangent;
    }
  } else if (point1Direction != AngleDirection::kPoint && point2Direction != AngleDirection::kPoint && point1Direction != point2Direction) {
    // Circle to circle and inner tangent
    // Find the point between these two circles
    QPointF midpoint;
    midpoint.setX(point1.x() + (point2.x()-point1.x())/2);
    midpoint.setY(point1.y() + (point2.y()-point1.y())/2);
    const double angleToTangent = angleBetweenCenterOfCircleAndIntersectionWithTangentLine(midpoint, point2, circleRadius);
    if (point1Direction == AngleDirection::kCounterclockwise) {
      angleBetweenPoints += angleToTangent;
    } else {
      angleBetweenPoints -= angleToTangent;
    }
  } else {
    // Point to point or circle to circle and outer tangent
  }

  return angleBetweenPoints;
}

void addSegmentToPath(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString) {
  // First, finish an arc if there is an open one
  if (previousApex.apexType != AngleDirection::kPoint && circleRadius > 0.0) {
    // Finish previously created arc
    //  We know that there must be at least one arc in the path
    PathSegment *mostRecentSegment = path.back().get();
    ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(mostRecentSegment);
    if (arc == nullptr) {
      throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
    }
    // A couple sanity checks, TODO: Remove after
    if (arc->circleCenter != previousApex.apexPoint) {
      throw std::runtime_error("arc->circleCenter != previousApex.apexPoint");
    } else if (arc->angleDirection != previousApex.apexType) {
      throw std::runtime_error("arc->angleDirection != previousApex.apexType");
    }
    arc->endAngle = math::angle(previousApex.apexPoint, edge.first);
  }

  // Second, add a straight segment between the apexes
  path.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(edge.first, edge.second)));

  if (newApex.apexType != AngleDirection::kPoint && circleRadius > 0.0) {
    // Finally, since this isn't the end, start a new arc
    path.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(newApex.apexPoint, circleRadius, newApex.apexType)));
    PathSegment *mostRecentSegment = path.back().get();
    ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(mostRecentSegment);
    if (arc == nullptr) {
      throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
    }
    arc->startAngle = math::angle(newApex.apexPoint, edge.second);
  }
}

std::pair<QPointF, QPointF> createCircleConciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius) {
  if (circleRadius < 0.001) {
    return {point1, point2};
  }
  QPointF lineStart, lineEnd;
  
  if (point1Direction == AngleDirection::kPoint) {
    lineStart = point1;
  }
  if (point2Direction == AngleDirection::kPoint) {
    lineEnd = point2;
  }
  
  if (point1Direction == AngleDirection::kPoint && point2Direction != AngleDirection::kPoint) {
    // Point to circle
    const auto intersectionPoints = intersectionsPointsOfTangentLinesToCircle(point1, point2, circleRadius);
    if (point2Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the left point
      lineEnd = intersectionPoints.first;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the right point
      lineEnd = intersectionPoints.second;
    }
  } else if (point1Direction != AngleDirection::kPoint && point2Direction == AngleDirection::kPoint) {
    // Circle to point
    const auto intersectionPoints = intersectionsPointsOfTangentLinesToCircle(point2, point1, circleRadius);
    if (point1Direction == AngleDirection::kClockwise) {
      // Agent will be turning clockwise around this point, get the angle of the right point (from the perspective of point2)
      lineStart = intersectionPoints.second;
    } else {
      // Agent will be turning counterclockwise around this point, get the angle of the left point (from the perspective of point2)
      lineStart = intersectionPoints.first;
    }
  } else if (point1Direction != AngleDirection::kPoint && point2Direction != AngleDirection::kPoint) {
    // Circle to circle
    if (point1Direction == point2Direction) {
      // Outer tangents
      double distanceBetweenCircles = math::distance(point1, point2);
      double absoluteAngleToCircle2 = math::angle(point1, point2);
      double distanceToNewPoint = sqrt(distanceBetweenCircles*distanceBetweenCircles + circleRadius*circleRadius);
      double angleFromCircle2ToNewPoint = asin(circleRadius / distanceToNewPoint);
      double newAngle = absoluteAngleToCircle2 + angleFromCircle2ToNewPoint;

      double dxFromPoint1 = distanceToNewPoint * cos(newAngle);
      double dyFromPoint1 = distanceToNewPoint * sin(newAngle);

      QPointF newPoint{point1.x() + dxFromPoint1, point1.y() + dyFromPoint1};
      double circleXOffset = newPoint.x() - point2.x();
      double circleYOffset = newPoint.y() - point2.y();

      if (point1Direction == AngleDirection::kClockwise) {
        // Both clockwise
        lineStart = QPointF{point1.x() + circleXOffset, point1.y() + circleYOffset};
        lineEnd = QPointF{point2.x() + circleXOffset, point2.y() + circleYOffset};
      } else {
        // Both counterclockwise
        lineStart = QPointF{point1.x() - circleXOffset, point1.y() - circleYOffset};
        lineEnd = QPointF{point2.x() - circleXOffset, point2.y() - circleYOffset};
      }
    } else {
      // Inner tangents
      // Find the point between these two circles
      QPointF midpoint;
      midpoint.setX(point1.x() + (point2.x()-point1.x())/2);
      midpoint.setY(point1.y() + (point2.y()-point1.y())/2);
      const auto intersectionPointsWithFirstCircle = intersectionsPointsOfTangentLinesToCircle(midpoint, point1, circleRadius);
      const auto intersectionPointsWithSecondCircle = intersectionsPointsOfTangentLinesToCircle(midpoint, point2, circleRadius);
      if (point1Direction == AngleDirection::kCounterclockwise) {
        // point2 must be clockwise
        lineStart = intersectionPointsWithFirstCircle.first;
        lineEnd = intersectionPointsWithSecondCircle.first;
      } else {
        // point1 is clockwise
        // point2 must be counterclockwise
        lineStart = intersectionPointsWithFirstCircle.second;
        lineEnd = intersectionPointsWithSecondCircle.second;
      }
    }
  }

  return {lineStart, lineEnd};
}

void addToLeftOfFunnel(std::deque<QPointF> &funnel, Apex &apex, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString) {
  auto newWedgeIsClockwiseToLeftmostWedge = [&funnel, &apex, &point, &circleRadius]() {
    if (funnel.size() < 2) {
      throw std::runtime_error("This function requires a funnel with at least two items");
    }
    if (funnel.front() == apex.apexPoint) {
      throw std::runtime_error("Assumption that the front of the funnel is not the apex was broken");
    }

    const QPointF leftmostEdgePoint1 = funnel.at(1);
    const QPointF leftmostEdgePoint2 = funnel.front();
    AngleDirection leftmostEdgePoint1Direction;
    const AngleDirection leftmostEdgePoint2Direction = AngleDirection::kCounterclockwise;

    // We know that funnel.front() is not the apex, but funnel.at(1) could be
    if (funnel.at(1) == apex.apexPoint) {
      leftmostEdgePoint1Direction = apex.apexType;
    } else {
      // Neither points are the apex
      leftmostEdgePoint1Direction = AngleDirection::kCounterclockwise;
    }

    const QPointF newEdgePoint1 = funnel.front();
    const QPointF newEdgePoint2 = point;
    const AngleDirection newEdgePoint1Direction = AngleDirection::kCounterclockwise;
    const AngleDirection newEdgePoint2Direction = AngleDirection::kCounterclockwise;

    const double theta = angle(leftmostEdgePoint1, leftmostEdgePoint1Direction, leftmostEdgePoint2, leftmostEdgePoint2Direction, circleRadius);
    const double phi = angle(newEdgePoint1, newEdgePoint1Direction, newEdgePoint2, newEdgePoint2Direction, circleRadius);
    return clockwiseTo(theta, phi); // Rotating from theta to phi is clockwise
  };

  // Make sure there is at least one edge in the funnel
  if (funnel.size() >= 2) {
    // Remove edges that are counterclockwise to this new potential edges
    while (funnel.front() != apex.apexPoint && newWedgeIsClockwiseToLeftmostWedge()) {
      funnel.pop_front();
    }

    // Need to check if this new left edge would cross over the apex

    while (funnel.front() == apex.apexPoint && funnel.size() >= 2) {
      std::pair<QPointF, QPointF> newWedge = createCircleConciousLine(funnel.front(), apex.apexType, point, AngleDirection::kCounterclockwise, circleRadius);
      std::pair<QPointF, QPointF> firstRightEdge = createCircleConciousLine(funnel.front(), apex.apexType, funnel.at(1), AngleDirection::kClockwise, circleRadius);
      if (math::crossProduct(newWedge.first, newWedge.second, firstRightEdge.first, firstRightEdge.second) > 0) {
        // New point crosses over apex
        if (math::distance(newWedge.first, newWedge.second) < math::distance(firstRightEdge.first, firstRightEdge.second)) {
          // New point is closer and should instead be the apex
          const Apex newApex{point, AngleDirection::kCounterclockwise};
          
          addSegmentToPath(apex, newWedge, newApex, path, circleRadius, pointToString);

          // Need to update left to be the apex
          funnel.pop_front();

          // Set new point as apex
          apex = newApex;
        } else {
          // Add the apex to the path
          const Apex newApex{funnel.at(1), AngleDirection::kClockwise};

          addSegmentToPath(apex, firstRightEdge, newApex, path, circleRadius, pointToString);

          // Remove apex from funnel
          funnel.pop_front();

          // Set end of right edge as current apex
          apex = newApex;
        }
      } else {
        // Done
        break;
      }
    }
  }
  // Finally, add point to left of funnel
  funnel.push_front(point);
}

void addToRightOfFunnel(std::deque<QPointF> &funnel, Apex &apex, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString, const bool isGoal) {
  auto newWedgeIsCounterclockwiseToRightmostWedge = [&funnel, &apex, &point, &circleRadius, &isGoal]() {
    if (funnel.size() < 2) {
      throw std::runtime_error("This function requires a funnel with at least two items");
    }
    if (funnel.back() == apex.apexPoint) {
      throw std::runtime_error("Assumption that the back of the funnel is not the apex was broken");
    }

    const QPointF rightmostEdgePoint1 = funnel.at(funnel.size()-2);
    const QPointF rightmostEdgePoint2 = funnel.back();
    AngleDirection rightmostEdgePoint1Direction;
    const AngleDirection rightmostEdgePoint2Direction = AngleDirection::kClockwise;

    // We know that funnel.back() is not the apex, but funnel.at(funnel.size()-2) could be
    if (funnel.at(funnel.size()-2) == apex.apexPoint) {
      rightmostEdgePoint1Direction = apex.apexType;
    } else {
      // Neither points are the apex
      rightmostEdgePoint1Direction = AngleDirection::kClockwise;
    }

    const QPointF newEdgePoint1 = funnel.back();
    const QPointF newEdgePoint2 = point;
    const AngleDirection newEdgePoint1Direction = AngleDirection::kClockwise;
    const AngleDirection newEdgePoint2Direction = (isGoal ? AngleDirection::kPoint : AngleDirection::kClockwise);

    const double theta = angle(rightmostEdgePoint1, rightmostEdgePoint1Direction, rightmostEdgePoint2, rightmostEdgePoint2Direction, circleRadius);
    const double phi = angle(newEdgePoint1, newEdgePoint1Direction, newEdgePoint2, newEdgePoint2Direction, circleRadius);
    return counterclockwiseTo(theta, phi); // Rotating from theta to phi is counterclockwise
  };

  // Make sure there is at least one edge in the funnel
  if (funnel.size() >= 2) {
    // Remove edges that are clockwise to this new potential edges
    while (funnel.back() != apex.apexPoint && newWedgeIsCounterclockwiseToRightmostWedge()) {
      funnel.pop_back();
    }

    // Need to check if this new right edge would cross over the apex
    while (funnel.back() == apex.apexPoint && funnel.size() >= 2) {
      std::pair<QPointF, QPointF> newWedge = createCircleConciousLine(funnel.back(), apex.apexType, point, (isGoal ? AngleDirection::kPoint : AngleDirection::kClockwise), circleRadius);
      std::pair<QPointF, QPointF> firstLeftEdge = createCircleConciousLine(funnel.back(), apex.apexType, funnel.at(funnel.size()-2), AngleDirection::kCounterclockwise, circleRadius);
      if (math::crossProduct(newWedge.first, newWedge.second, firstLeftEdge.first, firstLeftEdge.second) <= 0) {
        // New point crosses over apex
        if (math::distance(newWedge.first, newWedge.second) < math::distance(firstLeftEdge.first, firstLeftEdge.second)) {
          // New point is closer and should instead be the apex

          const Apex newApex{point, (isGoal ? AngleDirection::kPoint : AngleDirection::kClockwise)};
          
          addSegmentToPath(apex, newWedge, newApex, path, circleRadius, pointToString);

          // Need to update right to be the apex
          funnel.pop_back();

          // Set new point as apex
          apex = newApex;
        } else {
          // Add the apex to the path
          const Apex newApex{funnel.at(funnel.size()-2), AngleDirection::kCounterclockwise};

          addSegmentToPath(apex, firstLeftEdge, newApex, path, circleRadius, pointToString);

          // Remove apex from funnel
          funnel.pop_back();
          
          // Set end of left edge as current apex
          apex = newApex;
        }
      } else {
        // Done
        break;
      }
    }
  }
  // Finally, add point to right of funnel
  funnel.push_back(point);
}

void finishFunnel(std::deque<QPointF> &funnel, Apex &apex, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString) {
  // find index of apex
  int apexIndex=-1;
  for (int i=0; i<funnel.size(); ++i) {
    if (funnel.at(i) == apex.apexPoint) {
      apexIndex = i;
      break;
    }
  }
  if (apexIndex == -1) {
    throw std::runtime_error("Unable to find apex when adding goal");
  }
  AngleDirection t1 = apex.apexType;
  AngleDirection t2 = AngleDirection::kClockwise;
  for (int i=apexIndex; i<funnel.size()-1;) { // TODO: Think about this -1 in the condition
    if (i+1 == funnel.size()-1) {
      // Next point is the end of the last right edge
      t2 = AngleDirection::kPoint;
    }
    std::pair<QPointF, QPointF> newEdge = createCircleConciousLine(funnel.at(i), t1, funnel.at(i+1), t2, circleRadius);
    addSegmentToPath(Apex{funnel.at(i), t1}, newEdge, Apex{funnel.at(i+1), t2}, path, circleRadius, pointToString);
    t1 = AngleDirection::kClockwise;
    i += 1;
  }
}

double distanceBetweenEdgeAndPoint(const QPointF &edgeStartPoint, const QPointF &edgeEndPoint, const QPointF &point, QPointF *pointUsedForDistanceCalculation) {
  // TODO: Move to ::math
  const double dx = edgeEndPoint.x()-edgeStartPoint.x();
  const double dy = edgeEndPoint.y()-edgeStartPoint.y();
  const double lengthSquared = dx*dx+dy*dy;
  if (lengthSquared == 0.0) {
    // Line segment is just a point
    return math::distance(edgeStartPoint, point);
  }
  const double t = std::clamp(static_cast<double>(((point.x()-edgeStartPoint.x())*dx + (point.y()-edgeStartPoint.y())*dy) / lengthSquared), 0.0, 1.0);
  QPointF closestPoint{edgeStartPoint.x() + t*dx, edgeStartPoint.y() + t*dy};
  if (pointUsedForDistanceCalculation != nullptr) {
    *pointUsedForDistanceCalculation = closestPoint;
  }
  return math::distance(point, closestPoint);
}

bool operator==(const State &s1, const State &s2) {
  return (s1.isGoal && s2.isGoal) || (s1.triangleNum == s2.triangleNum && s1.entryEdge == s2.entryEdge);
}

bool operator<(const State &s1, const State &s2) {
  if (!s1.isGoal && !s2.isGoal) {
    if (s1.triangleNum == s2.triangleNum) {
      return s1.entryEdge < s2.entryEdge;
    } else {
      return s1.triangleNum < s2.triangleNum;
    }
  } else {
    return (s1.isGoal ? 1:0) < (s2.isGoal ? 1:0);
  }
}

std::ostream& operator<<(std::ostream& stream, const State &state) {
  if (state.isGoal) {
    stream << "[GOAL]";
  } else if (state.entryEdge == -1) {
    stream << "[START]";
  } else {
    stream << '(' << state.triangleNum << ',' << state.entryEdge << ')';
  }
  return stream;
}

void PathfindingResult::clear() {
  shortestPath.clear();
  aStarInfo.triangleCorridor.clear();
  aStarInfo.trianglesSearched.clear();
  aStarInfo.trianglesDiscovered.clear();
}