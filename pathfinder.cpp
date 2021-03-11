#include "debuglogger.h"
#include "pathfinder.h"
#include "vector.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <string>

std::vector<int> rebuildPath(State state, const std::map<State, State> &previous);

PathSegment::~PathSegment() {}

Pathfinder::Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData) : triangleData_(triangleData), triangleVoronoiData_(triangleVoronoiData) {
  // Initialize debug logger
  DebugLogger::instance().setPointToIndexFunction(std::bind(&Pathfinder::pointToIndex, std::cref(*this), std::placeholders::_1));

  // TODO: Start by checking that the triangle data is consistent
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

  // Given the triangle data, we will pre-compute the edges of each triangle
  triangleEdges_.resize(triangleData_.numberoftriangles);
  // Initialize all edges as -1
  std::for_each(triangleEdges_.begin(), triangleEdges_.end(), [](auto &arr) {
    arr[0] = -1;
    arr[1] = -1;
    arr[2] = -1;
  });
  auto addEdgeForTriangle = [](auto &edgeArray, const int edgeNumber) {
    if (edgeArray[0] == -1) {
      edgeArray[0] = edgeNumber;
    } else if (edgeArray[1] == -1) {
      edgeArray[1] = edgeNumber;
    } else if (edgeArray[2] == -1) {
      edgeArray[2] = edgeNumber;
    }
  };
  for (int voronoiEdgeNum=0;voronoiEdgeNum<triangleVoronoiData_.numberofedges; ++voronoiEdgeNum) {
    const int triangle1Number = triangleVoronoiData_.edgelist[voronoiEdgeNum*2];
    const int triangle2Number = triangleVoronoiData_.edgelist[voronoiEdgeNum*2+1];
    if (triangle1Number != -1) {
      addEdgeForTriangle(triangleEdges_.at(triangle1Number), voronoiEdgeNum);
    }
    if (triangle2Number != -1) {
      addEdgeForTriangle(triangleEdges_.at(triangle2Number), voronoiEdgeNum);
    }
  }
}

PathfindingResult Pathfinder::findShortestPath(const Vector &startPoint, const Vector &goalPoint) {
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

  // Check if it's even possible to reach the goal segment
  if (pathCanExist(startTriangle, goalTriangle)) {

    // It is possible to reach the goal, run A* to find the shortest path
    result.aStarInfo = triangleAStar(startPoint, startTriangle, goalPoint, goalTriangle);

    if (result.aStarInfo.triangleCorridor.empty()) {
      throw std::runtime_error("A* should've found a path");
    }

    // A path was found
    const auto corridor = buildCorridor(result.aStarInfo.triangleCorridor);
    PathFunnel pathFunnel(characterRadius_);
    pathFunnel.funnelWithGoal(corridor, startPoint, goalPoint);
    result.shortestPath = pathFunnel.getPath();
  }

  return result;
}

bool Pathfinder::pathCanExist(int startTriangle, int goalTriangle) {
  // Use breadth-first search to try to quickly find if a path exists between these two triangles

  std::set<int> visitedTriangles;
  // Use a set for a quicker lookup to check if we've already queued a State rather than overfilling the queue
  std::set<State> alreadyQueuedStates;
  std::queue<State> stateQueue;

  State startState;
  startState.triangleNum = startTriangle;

  stateQueue.push(startState);
  alreadyQueuedStates.insert(startState);

  while (!stateQueue.empty()) {
    State currentState = stateQueue.front();
    stateQueue.pop();

    const auto successors = getSuccessors(currentState, goalTriangle);
    for (const auto &successorState : successors) {
      if (successorState.triangleNum == goalTriangle) {
        // Found a path to the goal
        return true;
      }
      if (visitedTriangles.find(successorState.triangleNum) == visitedTriangles.end()) {
        // Only care about triangles that we havent visited
        // TODO: Definitely this should be States rather than triangles (bridges and the area below them will share triangles)
        if (alreadyQueuedStates.find(successorState) == alreadyQueuedStates.end()) {
          // Not yet queued, queue it
          stateQueue.push(successorState);
          alreadyQueuedStates.insert(successorState);
        }
      }
    }
    // Mark current triangle as visited
    visitedTriangles.insert(currentState.triangleNum);
  }

  // Never found the goal triangle, must be impossible to reach
  return false;
}

int Pathfinder::getSharedEdge(const int triangle1Num, const int triangle2Num) const {
  if (triangle1Num == -1 || triangle2Num == -1) {
    throw std::runtime_error("Trying to get shared edge for non-existent triangle");
  }
  const auto &triangle1Edges = triangleEdges_.at(triangle1Num);
  const auto &triangle2Edges = triangleEdges_.at(triangle2Num);
  for (const int triangle1EdgeNum : triangle1Edges) {
    if (triangle1EdgeNum != -1) {
      for (const int triangle2EdgeNum : triangle2Edges) {
        if (triangle2EdgeNum != -1) {
          if (triangle1EdgeNum == triangle2EdgeNum) {
            // Found the shared edge
            return triangle1EdgeNum;
          }
        }
      }
    }
  }
  return -1;
}

std::vector<std::pair<Vector,Vector>> Pathfinder::buildCorridor(const std::vector<int> &trianglesInCorridor) const {
  std::vector<std::pair<Vector,Vector>> corridorSegments;
  for (int i=1; i<trianglesInCorridor.size(); ++i) {
    // find common edge between triangle i,i-1
    const int triangleANum = trianglesInCorridor.at(i-1);
    const int triangleBNum = trianglesInCorridor.at(i);
    if (triangleANum >= triangleData_.numberoftriangles || triangleBNum >= triangleData_.numberoftriangles) {
      throw std::runtime_error("Invalid triangle number while building corridor");
    }
    int sharedEdge = getSharedEdge(triangleANum, triangleBNum);
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
    const Vector sharedEdgeVertexA{triangleData_.pointlist[sharedEdgeVertexAIndex*2], triangleData_.pointlist[sharedEdgeVertexAIndex*2+1]};
    const Vector sharedEdgeVertexB{triangleData_.pointlist[sharedEdgeVertexBIndex*2], triangleData_.pointlist[sharedEdgeVertexBIndex*2+1]};
    corridorSegments.emplace_back(sharedEdgeVertexA, sharedEdgeVertexB);
  }
  return corridorSegments;
}

int Pathfinder::pointToIndex(const Vector &point) const {
  // TODO: Remove once done debugging
  for (int i=0; i<triangleData_.numberofpoints; ++i) {
    if (point.x() == triangleData_.pointlist[i*2] && point.y() == triangleData_.pointlist[i*2+1]) {
      return i;
    }
  }
  throw std::runtime_error("Unknown point");
}

void Pathfinder::setCharacterRadius(double value) {
  characterRadius_ = value;
}

bool Pathfinder::pointIsInTriangle(const Vector &point, const int triangleNum) const {
  const int vertexIndexA = triangleData_.trianglelist[triangleNum*3];
  const int vertexIndexB = triangleData_.trianglelist[triangleNum*3+1];
  const int vertexIndexC = triangleData_.trianglelist[triangleNum*3+2];
  if (vertexIndexA >= triangleData_.numberofpoints ||
      vertexIndexB >= triangleData_.numberofpoints ||
      vertexIndexC >= triangleData_.numberofpoints) {
    throw std::runtime_error("Triangle references vertex which does not exist");
  }
  const Vector vertexA{triangleData_.pointlist[vertexIndexA*2], triangleData_.pointlist[vertexIndexA*2+1]};
  const Vector vertexB{triangleData_.pointlist[vertexIndexB*2], triangleData_.pointlist[vertexIndexB*2+1]};
  const Vector vertexC{triangleData_.pointlist[vertexIndexC*2], triangleData_.pointlist[vertexIndexC*2+1]};

  // Triangles' vertices are listed in CCW order (might matter for checking if a point lies within a triangle)
  return math::isPointInTriangle(point, vertexA, vertexB, vertexC);
}

int Pathfinder::findTriangleForPoint(const Vector &point) const {
  if (triangleData_.trianglelist == nullptr) {
    throw std::runtime_error("Triangle list is null!");
  }
  if (triangleData_.pointlist == nullptr) {
    throw std::runtime_error("Point list is null!");
  }
  // TODO: Check for more nulls in lists

  // Loop over all triangles
  for (int triangleNumber=0; triangleNumber<triangleData_.numberoftriangles; ++triangleNumber) {
    if (pointIsInTriangle(point, triangleNumber)) {
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
  
  const Vector edge1VertexA{triangleData_.pointlist[edge1VertexAIndex*2], triangleData_.pointlist[edge1VertexAIndex*2+1]};
  const Vector edge1VertexB{triangleData_.pointlist[edge1VertexBIndex*2], triangleData_.pointlist[edge1VertexBIndex*2+1]};
  const Vector edge2VertexA{triangleData_.pointlist[edge2VertexAIndex*2], triangleData_.pointlist[edge2VertexAIndex*2+1]};
  const Vector edge2VertexB{triangleData_.pointlist[edge2VertexBIndex*2], triangleData_.pointlist[edge2VertexBIndex*2+1]};
  const double angle = math::angleBetweenVectors(edge1VertexA, edge1VertexB, edge2VertexA, edge2VertexB);
  // TODO: Sometimes this arclength causes weird paths
  // return 0;
  // return 0.1 * characterRadius_ * angle;
  return characterRadius_ * angle; // Original from paper
}

double Pathfinder::calculateEstimateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<State, double> &gScores) const {
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

std::tuple<double, Vector, std::optional<LengthFunnel>> Pathfinder::calculateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<State, State> &previous) const {
  // The common edge between triangles `state` and `parentState`
  const int commonEdgeNum = state.entryEdge;

  if (state.isGoal) {
    // Find length of actual path from start to goal
    double result;
    std::vector<int> triangleCorridor = rebuildPath(parentState, previous);
    
    if (triangleCorridor.size() == 1) {
      // Only one triangle in corridor, result is a straight line
      result = math::distance(startPoint, goalPoint);
      // std::cout << "  [G Value] (" << result << ") Straight line from start to goal" << std::endl; //DEBUGPRINTS
    } else {
      // Multiple triangles in corridor, do normal funnel algorithm to find the exact path length
      // std::cout << "Funneling to state " << state << std::endl; //DEBUGPRINTS

      LengthFunnel lengthFunnel(characterRadius_);
      const auto funnelCacheIt = lengthFunnelCache_.find(parentState);
      if (funnelCacheIt != lengthFunnelCache_.end()) {
        // We have a cached funnel for the parent state, lets copy that (we know it already has enough space for the goal)
        lengthFunnel = funnelCacheIt->second;
      } else {
        // Don't have a cached funnel, build the whole thing
        const auto corridor = buildCorridor(triangleCorridor);
        lengthFunnel.funnelWithoutGoal(corridor, startPoint);
      }

      // Finish the funnel with the goal
      lengthFunnel.finishFunnelWithGoal(goalPoint);
      result = lengthFunnel.getLength();
      // std::cout << "  [G Value] (" << result << "), " << shortestPath.size() << "-segment path from start to goal" << std::endl; //DEBUGPRINTS
    }
    // Never caching the funnel used here
    return {result, goalPoint, std::optional<LengthFunnel>{}};
  }

  // State is not the goal
  double result;
  Vector pointUsed;
  std::optional<LengthFunnel> optionalFunnelForCaching;
  std::vector<int> triangleCorridor = rebuildPath(parentState, previous);
  // Add the current triangle to the corridor
  triangleCorridor.emplace_back(state.triangleNum);

  if (triangleCorridor.size() == 1) {
    // Only one triangle in corridor
    result = distanceBetweenEdgeAndPoint(commonEdgeNum, startPoint, &pointUsed);
    // std::cout << "  [G Value] (" << result << ") Straight line from start to edge " << commonEdgeNum << std::endl; //DEBUGPRINTS
  } else {
    // Funnel from startPoint to a point that minimizes g-value
    // std::cout << "Funneling to state " << state << std::endl; //DEBUGPRINTS

    // ============================================================
    LengthFunnel lengthFunnel(characterRadius_);
    const auto commonEdge = getEdge(commonEdgeNum);
    const auto funnelCacheIt = lengthFunnelCache_.find(parentState);
    if (funnelCacheIt != lengthFunnelCache_.end()) {
      // We have a cached funnel for the parent state, lets copy that and extend it
      // Duplicate the cached funnel, but make sure that we have room for our new edge
      lengthFunnel = funnelCacheIt->second.cloneFunnelButSpaceFor1MorePoint();


      // Add the next edge in the corriror (which is the edge betweeen 'parentState' and 'state')
      lengthFunnel.extendByOneEdge(commonEdge);
    } else {
      // Don't have a cached funnel, build the whole thing
      const auto corridor = buildCorridor(triangleCorridor);
      lengthFunnel.funnelWithoutGoal(corridor, startPoint);
    }

    // Save the funnel before adding the goal
    optionalFunnelForCaching = lengthFunnel;

    // Figure out the shortest path to get to 'state'
    pointUsed = lengthFunnel.finishFunnelAndFindClosestGoalOnEdge(commonEdge);
    
    // std::cout << "Shortest path: ["; //DEBUGPRINTS
    // for (const auto &i : shortestPath) { //DEBUGPRINTS
      // const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(i.get()); //DEBUGPRINTS
      // const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(i.get()); //DEBUGPRINTS
      // if (straightSegment != nullptr) { //DEBUGPRINTS
        // std::cout << "straight,"; //DEBUGPRINTS
      // } else if (arcSegment != nullptr) { //DEBUGPRINTS
        // std::cout << "arc,"; //DEBUGPRINTS
      // } //DEBUGPRINTS
    // } //DEBUGPRINTS
    // std::cout << "]" << std::endl; //DEBUGPRINTS

    result = lengthFunnel.getLength();
    // std::cout << "  Funneling for g(x) from start to edge " << commonEdgeNum << ", length: " << result << std::endl; //DEBUGPRINTS
  }
  return {result, pointUsed, optionalFunnelForCaching};
}

Vector Pathfinder::midpointOfEdge(int edgeNum) const {
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
  const Vector vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  const Vector vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};
  const auto dx = vertexB.x()-vertexA.x();
  const auto dy = vertexB.y()-vertexA.y();
  return Vector{vertexA.x()+dx/2, vertexB.y()+dy/2};
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
  const Vector vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  const Vector vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};
  return math::distance(vertexA, vertexB);
}

std::pair<Vector,Vector> Pathfinder::getEdge(int edgeNum) const {
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
  const Vector vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  const Vector vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};
  return {vertexA, vertexB};
}

double Pathfinder::distanceBetweenEdgeAndPoint(int edgeNum, const Vector &point, Vector *pointUsedForDistanceCalculation) const {
  // Return the distance between the closest end of the given edge and the given point
  const auto [vertexA, vertexB] = getEdge(edgeNum);
  return math::distanceBetweenEdgeAndPoint(vertexA, vertexB, point, pointUsedForDistanceCalculation);
}

double Pathfinder::calculateHValue(const State &state, const Vector &goalPoint) const {
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
    
    const Vector entryEdgeVertexA{triangleData_.pointlist[entryEdgeVertexAIndex*2], triangleData_.pointlist[entryEdgeVertexAIndex*2+1]};
    const Vector entryEdgeVertexB{triangleData_.pointlist[entryEdgeVertexBIndex*2], triangleData_.pointlist[entryEdgeVertexBIndex*2+1]};
    const Vector exitEdgeVertexA{triangleData_.pointlist[exitEdgeVertexAIndex*2], triangleData_.pointlist[exitEdgeVertexAIndex*2+1]};
    const Vector exitEdgeVertexB{triangleData_.pointlist[exitEdgeVertexBIndex*2], triangleData_.pointlist[exitEdgeVertexBIndex*2+1]};

    return ((math::distance(entryEdgeVertexA, entryEdgeVertexB) >= (characterRadius_*2)) && math::distance(exitEdgeVertexA, exitEdgeVertexB) >= (characterRadius_*2));
    // TODO: This is a better approximation until the below code handles unconstrained edges

    Vector sharedPoint, oppositeEdgeStart, oppositeEdgeEnd;

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

    return (math::distanceBetweenEdgeAndPoint(oppositeEdgeStart, oppositeEdgeEnd, sharedPoint) >= (characterRadius_*2));
  };
  
  // For each neighboring triangle index
  for (int neighborNum=state.triangleNum*3; neighborNum<state.triangleNum*3+3; ++neighborNum) {
    const auto neighborTriangleNum = triangleData_.neighborlist[neighborNum];
    if (neighborTriangleNum != -1) {
      // Neighbor exists
      int sharedEdge = getSharedEdge(state.triangleNum, neighborTriangleNum);
      if (sharedEdge != -1 && sharedEdge != state.entryEdge) {
        // Is a valid edge and isn't the entry edge
        if (!(kAvoidObstacles && triangleData_.edgemarkerlist[sharedEdge] != 0)) {
          // Non-constraint edge
          if (state.entryEdge < 0 || characterFitsThroughTriangle(state.entryEdge, sharedEdge)) {
            State successor;
            successor.entryEdge = sharedEdge;
            successor.triangleNum = neighborTriangleNum;
            result.push_back(successor);
          }
        }
      }
    }
  }
  return result;
}


PathfindingAStarInfo Pathfinder::triangleAStar(const Vector &startPoint, int startTriangle, const Vector &goalPoint, int goalTriangle) const {
  PathfindingAStarInfo result;
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Trying to find shortest path from triangle " << startTriangle << " to triangle " << goalTriangle << std::endl; //DEBUGPRINTS
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  std::vector<State> openSet; // "open set"
  std::set<State> visited;

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

  
  double shortestPathLength = std::numeric_limits<double>::max();
  State finalGoalState;
  bool found = false;

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
    
    // std::cout << "We chose " << currentState << " as our next candidate" << std::endl; //DEBUGPRINTS
    openSet.erase(minElementIt);

    if (currentState.isGoal) {
      // Found the goal
      result.triangleCorridor = rebuildPath(currentState, previous);
      return result;
    }

    // Look at successors
    auto successors = getSuccessors(currentState, goalTriangle);
    for (const auto &successor : successors) {
      // std::cout << "  -Possible successor- " << successor << std::endl; //DEBUGPRINTS
      if (visited.find(successor) == visited.end()) {
        // Touched this successor
        result.trianglesDiscovered.emplace(successor.triangleNum);

        // std::cout << "  -Evaluatating successor- " << successor << std::endl; //DEBUGPRINTS
        if (gScores.find(successor) == gScores.end()) {
          // No gScore yet for this state, initialize
          gScores.emplace(successor, std::numeric_limits<double>::max());
        }
        if (fScores.find(successor) == fScores.end()) {
          // No fScore yet for this state, initialize
          fScores.emplace(successor, std::numeric_limits<double>::max());
        }

        const bool kOptimalPath{true};
        if (kOptimalPath) {
          // The heuristic scale factor allows this A* algorithm to be tailored more towards:
          //  1. Djikstra's search
          //    a. A scale factor < 1 will result in a potentially wider search area
          //    b. Will trend towards a globally optimal path when the heuristic is not admissible
          //  2. Greedy Best First search
          //    a. A scale factor > 1 will result in a more direct but potentially incorrect search
          const double kHeuristicScaleFactor = 0.50;
          const auto [gValueOfSuccessor, pointUsedForGValue, optionalFunnelCreated] = calculateGValue(successor, currentState, startPoint, goalPoint, previous);
          const double oldHValue = calculateHValue(successor, goalPoint);
          const double newHValue = math::distance(pointUsedForGValue, goalPoint);
          const double hValueOfSuccessor = kHeuristicScaleFactor * std::min(oldHValue, newHValue);
          const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
          // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f (old: %.9f, new: %.9f), and fScore: %.9f\n", successor.entryEdge, gValueOfSuccessor, hValueOfSuccessor, oldHValue, newHValue, fValueOfSuccessor); //DEBUGPRINTS
          if (math::lessThan(fValueOfSuccessor, fScores.at(successor))) {

            if (fScores.at(successor) != std::numeric_limits<double>::max()) {
              // std::cout << "    Better fscore" << std::endl; //DEBUGPRINTS
            }
            // Update previous
            previous[successor] = currentState;

            if (optionalFunnelCreated) {
              // Funnel was created, cache it
              // May overwrite existing cached funnel, that's intentional
              if (fScores.at(successor) != std::numeric_limits<double>::max()) {
                // std::cout << "    Found a better fscore for state " << successor << ", overwriting cached funnel" << std::endl; //DEBUGPRINTS
              } else {
                // std::cout << "    Caching first funnel for state " << successor << std::endl; //DEBUGPRINTS
              }
              lengthFunnelCache_[successor] = *optionalFunnelCreated;
            }

            // Update fScore
            fScores.at(successor) = fValueOfSuccessor;

            // Add to open set if not already in
            if (std::find(openSet.begin(), openSet.end(), successor) == openSet.end()) {
              openSet.push_back(successor);
            }
          }
        } else {
          const double gValueOfSuccessor = calculateEstimateGValue(successor, currentState, startPoint, goalPoint, gScores);
          const double hValueOfSuccessor = calculateHValue(successor, goalPoint);
          const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
          // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f, and fScore: %.9f\n", successor.entryEdge, gValueOfSuccessor, hValueOfSuccessor, fValueOfSuccessor); //DEBUGPRINTS
          if (math::lessThan(fValueOfSuccessor, fScores.at(successor))) {
            // std::cout << "    Better fscore" << std::endl; //DEBUGPRINTS
            // Update previous
            previous[successor] = currentState;

            // Update gScore
            gScores.at(successor) = gValueOfSuccessor;

            // Update fScore
            fScores.at(successor) = fValueOfSuccessor;

            // Add to open set if not already in
            if (std::find(openSet.begin(), openSet.end(), successor) == openSet.end()) {
              openSet.push_back(successor);
            }
          }
        }
      }
    }
    // std::cout << std::endl; //DEBUGPRINTS

    // Completely evaluated this state
    visited.insert(currentState);
    result.trianglesSearched.emplace(currentState.triangleNum);
  }

  // Open set is empty, but no path was found
  // std::cout << std::endl; //DEBUGPRINTS
  return result;
}

/* ========================================================================== **
** ============================= Free functions ============================= **
** ========================================================================== */

double calculatePathLength(const std::vector<std::unique_ptr<PathSegment>> &path) {
  // std::cout << "    calculatePathLength: "; //DEBUGPRINTS
  double totalDistance=0;
  for (int i=0; i<path.size(); ++i) {
    const PathSegment *segment = path.at(i).get();
    const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(segment);
    const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(segment);
    if (straightSegment != nullptr) {
      totalDistance += math::distance(straightSegment->startPoint, straightSegment->endPoint);
      // std::cout << "+straight [" << totalDistance << "], "; //DEBUGPRINTS
    } else if (arcSegment != nullptr) {
      if (i != path.size()-1) {
        // If the last segment is an arc, i think its probably an unfinished path and we shouldnt add this segment
        double angle = math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, arcSegment->angleDirection);
        totalDistance += arcSegment->circleRadius * abs(angle);
        // std::cout << "+arc (angle start:" << arcSegment->startAngle << ", angle end: " << arcSegment->endAngle << ", angle: " << angle << ", radius:" << arcSegment->circleRadius << ") [" << totalDistance << "], "; //DEBUGPRINTS
      }
    }
  }
  // std::cout << std::endl; //DEBUGPRINTS
  return totalDistance;
}

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