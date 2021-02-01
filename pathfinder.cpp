#include "math_helpers.h"
#include "pathfinder.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <string>

std::vector<int> rebuildPath(State state, const std::map<State, State> &previous);
void addToLeftOfFunnel(std::deque<QPointF> &funnel, QPointF &apex, const QPointF &point, std::vector<QPointF> &path);
void addToRightOfFunnel(std::deque<QPointF> &funnel, QPointF &apex, const QPointF &point, std::vector<QPointF> &path);
void finishFunnel(std::deque<QPointF> &funnel, QPointF &apex, std::vector<QPointF> &path);

Pathfinder::Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData) : triangleData_(triangleData), triangleVoronoiData_(triangleVoronoiData) {}

PathfindingResult Pathfinder::findShortestPath(const QPointF &startPoint, const QPointF &goalPoint) {
  int startTriangle = findTriangleForPoint(startPoint);
  int goalTriangle = findTriangleForPoint(goalPoint);
  PathfindingResult result;
  if (startTriangle == goalTriangle) {
    // Only one triangle involved
    // Shortest path is a straight line between start and goal
    result.shortestPath = std::vector<QPointF>{startPoint, goalPoint};
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

std::vector<QPointF> Pathfinder::funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, const QPointF &goalPoint) const {
  if (trianglesInCorridor.size() == 1) {
    // Start and goal are in the same triangle, the path is a straight line from start to goal
    return {startPoint, goalPoint};
  }
  // Build a list of "edges" that we'll need to pass through in this corridor
  using CorridorType = std::vector<std::pair<QPointF,QPointF>>;
  using PathType = std::vector<QPointF>;
  CorridorType corridor = buildCorridor(trianglesInCorridor);
  PathType path;
  QPointF currentApex;
  
  // Start with the start point on the path
  path.emplace_back(startPoint);

  std::deque<QPointF> funnel;
  // Initialize funnel with start point
  funnel.push_front(startPoint);
  // Initialize apex as start point
  currentApex = startPoint;

  // Start by adding
  QPointF left, right;
  // First and second point are the two edges of the first edge
  std::tie(left, right) = corridor.at(0);
  const bool areCorrectOrder = (math::crossProduct(startPoint, right, startPoint, left) > 0);
  if (!areCorrectOrder) {
    // incorrect order (left is actually right), swap
    std::swap(left, right);
  }
  addToLeftOfFunnel(funnel, currentApex, left, path);
  addToRightOfFunnel(funnel, currentApex, right, path);

  for (int i=1; i<corridor.size(); ++i) {
    QPointF newPoint1, newPoint2;
    std::tie(newPoint1, newPoint2) = corridor.at(i);
    if (newPoint1 == right) {
      // newPoint2 will be left
      left = newPoint2;
      addToLeftOfFunnel(funnel, currentApex, left, path);
    } else if (newPoint1 == left) {
      // newPoint2 will be right
      right = newPoint2;
      addToRightOfFunnel(funnel, currentApex, right, path);
    } else if (newPoint2 == right) {
      // newPoint1 will be left
      left = newPoint1;
      addToLeftOfFunnel(funnel, currentApex, left, path);
    } else {
      // newPoint1 will be right
      right = newPoint1;
      addToRightOfFunnel(funnel, currentApex, right, path);
    }
  }

  // Finally, add the goal to the right of the funnel
  addToRightOfFunnel(funnel, currentApex, goalPoint, path);
  // And finish the algorithm, closing out the funnel
  finishFunnel(funnel, currentApex, path);
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
  const double dotProduct = math::dotProduct(edge1VertexA, edge1VertexB, edge2VertexA, edge2VertexB);
  const double lengthsMultiplied = math::distance(edge1VertexA, edge1VertexB) * math::distance(edge2VertexA, edge2VertexB);
  return characterRadius_ * std::acos(dotProduct/lengthsMultiplied);
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
  const double dx = vertexB.x()-vertexA.x();
  const double dy = vertexB.y()-vertexA.y();
  return sqrt(dx*dx+dy*dy);
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
  const double dx = vertexB.x()-vertexA.x();
  const double dy = vertexB.y()-vertexA.y();
  const double lengthSquared = dx*dx+dy*dy;
  if (lengthSquared == 0.0) {
    // Line segment is just a point
    return math::distance(vertexA, point);
  }
  const double t = std::clamp(static_cast<double>(((point.x()-vertexA.x())*dx + (point.y()-vertexA.y())*dy) / lengthSquared), 0.0, 1.0);
  QPointF closestPoint{vertexA.x() + t*dx, vertexA.y() + t*dy};
  if (pointUsedForDistanceCalculation != nullptr) {
    *pointUsedForDistanceCalculation = closestPoint;
  }
  return math::distance(point, closestPoint);
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
  
  // For each neighboring triangle index
  for (int neighborNum=state.triangleNum*3; neighborNum<state.triangleNum*3+3; ++neighborNum) {
    const auto neighborTriangleNum = triangleData_.neighborlist[neighborNum];
    if (neighborTriangleNum != -1) {
      // Neighbor exists
      // Find out which edge is shared between this neighbor and the original triangle
      // TODO: Optimize, probably shouldnt need to loop through all edges
      for (int edgeNum=0; edgeNum<triangleVoronoiData_.numberofedges; ++edgeNum) {
        const int triangle1Number = triangleVoronoiData_.edgelist[edgeNum*2];
        const int triangle2Number = triangleVoronoiData_.edgelist[edgeNum*2+1];
        if (!((kCareAboutCharacterRadius && characterRadius_ != 0.0) && lengthOfEdge(edgeNum) < characterRadius_*2) && !(kAvoidObstacles && triangleData_.edgemarkerlist[edgeNum] != 0)) {
          // Non-constraint edge
          if ((triangle1Number == state.triangleNum &&
              triangle2Number == neighborTriangleNum) ||
              (triangle1Number == neighborTriangleNum &&
              triangle2Number == state.triangleNum)) {
            // Found the shared edge
            if (edgeNum != state.entryEdge) {
              // Only add a successor if it isnt the previous triangle
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

void addToLeftOfFunnel(std::deque<QPointF> &funnel, QPointF &apex, const QPointF &point, std::vector<QPointF> &path) {
  while (true) {
    bool isCounterClockwise = false;
    if (funnel.size() <= 1) {
      funnel.push_front(point);
      break;
    } else if (funnel.front() == apex) {
      // Compare angle of first right wedge (from apex) to this potential new left wedge
      const auto &firstRightWedgePoint1 = funnel.front();
      const auto &firstRightWedgePoint2 = funnel.at(1);
      const auto &newLeftWedgePoint1 = funnel.front();
      const auto &newLeftWedgePoint2 = point;
      isCounterClockwise = (math::crossProduct(firstRightWedgePoint1, firstRightWedgePoint2, newLeftWedgePoint1, newLeftWedgePoint2) > 0);
    } else {
      // Compare the angle of the most recent left wedge to this potential new left wedge
      const auto &mostRecentLeftWedgePoint1 = funnel.at(1);
      const auto &mostRecentLeftWedgePoint2 = funnel.front();
      const auto &newLeftWedgePoint1 = funnel.front();
      const auto &newLeftWedgePoint2 = point;
      isCounterClockwise = (math::crossProduct(mostRecentLeftWedgePoint1, mostRecentLeftWedgePoint2, newLeftWedgePoint1, newLeftWedgePoint2) > 0);
    }
    if (isCounterClockwise) {
      funnel.push_front(point);
      break;
    }
    if (funnel.front() == apex) {
      // Crosses over old apex, time to move apex one to the right
      funnel.pop_front();
      apex = funnel.front();
      path.emplace_back(apex);
      funnel.push_front(point);
    }
    funnel.pop_front();
  }
}

void addToRightOfFunnel(std::deque<QPointF> &funnel, QPointF &apex, const QPointF &point, std::vector<QPointF> &path) {
  while (true) {
    bool isClockwise = false;
    if (funnel.size() <= 1) {
      funnel.push_back(point);
      break;
    } else if (funnel.back() == apex) {
      // Compare angle of first left wedge (from apex) to this potential new right wedge
      const auto &firstLeftWedgePoint1 = funnel.back();
      const auto &firstLeftWedgePoint2 = funnel.at(funnel.size()-2);
      const auto &newRightWedgePoint1 = funnel.back();
      const auto &newRightWedgePoint2 = point;
      isClockwise = (math::crossProduct(firstLeftWedgePoint1, firstLeftWedgePoint2, newRightWedgePoint1, newRightWedgePoint2) < 0);
    } else {
      // Compare the angle of the most recent right wedge to this potential new right wedge
      const auto &mostRecentRightWedgePoint1 = funnel.at(funnel.size()-2);
      const auto &mostRecentRightWedgePoint2 = funnel.back();
      const auto &newRightWedgePoint1 = funnel.back();
      const auto &newRightWedgePoint2 = point;
      isClockwise = (math::crossProduct(mostRecentRightWedgePoint1, mostRecentRightWedgePoint2, newRightWedgePoint1, newRightWedgePoint2) < 0);
    }
    if (isClockwise) {
      funnel.push_back(point);
      break;
    }
    if (funnel.back() == apex) {
      // Crosses over old apex, time to move apex one to the left
      funnel.pop_back();
      apex = funnel.back();
      path.emplace_back(apex);
      funnel.push_back(point);
    }
    funnel.pop_back();
  }
}

void finishFunnel(std::deque<QPointF> &funnel, QPointF &apex, std::vector<QPointF> &path) {
  // find index of apex
  int apexIndex=-1;
  for (int i=0; i<funnel.size(); ++i) {
    if (funnel.at(i) == apex) {
      apexIndex = i;
      break;
    }
  }
  if (apexIndex == -1) {
    throw std::runtime_error("Unable to find apex when adding goal");
  }
  for (int i=apexIndex; i<funnel.size() && funnel.at(i)!=funnel.back();) {
    i += 1;
    path.emplace_back(funnel.at(i));
  }
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