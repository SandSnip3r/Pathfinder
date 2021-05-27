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

namespace pathfinder {

std::vector<int> rebuildPath(navmesh::State state, const std::map<navmesh::State, navmesh::State> &previous);

PathSegment::~PathSegment() {}

Pathfinder::Pathfinder(const navmesh::AStarNavmeshInterface &navmesh, const double agentRadius) : navmesh_(navmesh), agentRadius_(agentRadius) {
  // Initialize debug logger

  DebugLogger::instance().setPointToIndexFunction(std::bind(&navmesh::AStarNavmeshInterface::getVertexIndex, std::cref(navmesh_), std::placeholders::_1));
}

PathfindingResult Pathfinder::findShortestPath(const Vector &startPoint, const Vector &goalPoint) const {
  lengthFunnelCache_.clear();
  int startTriangle = navmesh_.findTriangleForPoint(startPoint);
  if (collidesWithConstraint(startPoint, startTriangle)) {
    throw std::runtime_error("The chosen start point is overlapping with a constraint. Pathing not possible");
  }
  int goalTriangle = navmesh_.findTriangleForPoint(goalPoint);
  if (collidesWithConstraint(goalPoint, goalTriangle)) {
    throw std::runtime_error("The chosen goal point is overlapping with a constraint. Pathing not possible");
  }
  PathfindingResult result;
  if (startTriangle == goalTriangle) {
    // Only one triangle involved
    buildShortestPathWithinSingleTriangle(startTriangle, startPoint, goalPoint, result.shortestPath);
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
    PathFunnel pathFunnel(agentRadius_);
    pathFunnel.funnelWithGoal(corridor, startPoint, goalPoint);
    result.shortestPath = pathFunnel.getPath();
  }

  // In the case that no path exists, the result is empty
  return result;
}

bool Pathfinder::pathCanExist(int startTriangle, int goalTriangle) const {
  // Use breadth-first search to try to quickly find if a path exists between these two triangles

  std::set<int> visitedTriangles;
  // Use a set for a quicker lookup to check if we've already queued a State rather than overfilling the queue
  std::set<navmesh::State> alreadyQueuedStates;
  std::queue<navmesh::State> stateQueue;

  navmesh::State startState(startTriangle);

  stateQueue.push(startState);
  alreadyQueuedStates.insert(startState);

  while (!stateQueue.empty()) {
    navmesh::State currentState = stateQueue.front();
    stateQueue.pop();

    const auto successors = navmesh_.getSuccessors(currentState, goalTriangle, agentRadius_);
    for (const auto &successorState : successors) {
      if (successorState.getTriangleIndex() == goalTriangle) {
        // Found a path to the goal
        return true;
      }
      if (visitedTriangles.find(successorState.getTriangleIndex()) == visitedTriangles.end()) {
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
    visitedTriangles.insert(currentState.getTriangleIndex());
  }

  // Never found the goal triangle, must be impossible to reach
  return false;
}

std::vector<std::pair<Vector,Vector>> Pathfinder::buildCorridor(const std::vector<int> &trianglesInCorridor) const {
  std::vector<std::pair<Vector,Vector>> corridorSegments;
  int triangle1Index, triangle2Index;
  for (int i=1; i<trianglesInCorridor.size(); ++i) {
    // find common edge between triangle i,i-1
    triangle1Index = trianglesInCorridor.at(i-1);
    triangle2Index = trianglesInCorridor.at(i);
    const auto sharedEdge = navmesh_.getSharedEdge(triangle1Index, triangle2Index);
    corridorSegments.emplace_back(sharedEdge.first, sharedEdge.second);
  }
  return corridorSegments;
}

bool Pathfinder::collidesWithConstraint(const Vector &point, const int triangleIndex) const {
  if (triangleIndex < 0) {
    throw std::runtime_error("Trying to check for collision for non-existent triangle ("+std::to_string(triangleIndex)+")");
  }

  const auto &verticesIndices = navmesh_.getTriangleVertexIndices(triangleIndex);

  for (const int vertexIndex : {std::get<0>(verticesIndices), std::get<1>(verticesIndices), std::get<2>(verticesIndices)}) {
    const auto marker = navmesh_.getVertexMarker(vertexIndex);
    if (marker != 0) {
      // Constraint vertex
      const auto &vertex = navmesh_.getVertex(vertexIndex);
      if (math::lessThan(math::distance(point, vertex), agentRadius_)) {
        // Point is too close to vertex
        return true;
      }
    }
  }
  return false;
}

double Pathfinder::calculateArcLength(const int edge1Index, const int edge2Index) const {
  // Return the length of the arc from one edge to another (they share a vertex)
  const auto &[edge1Vertex1, edge1Vertex2] = navmesh_.getEdge(edge1Index);
  const auto &[edge2Vertex1, edge2Vertex2] = navmesh_.getEdge(edge1Index);
  const double angle = math::angleBetweenVectors(edge1Vertex1, edge1Vertex2, edge2Vertex1, edge2Vertex2);
  // TODO: Sometimes this arclength causes weird paths
  // return 0;
  // return 0.1 * agentRadius_ * angle;
  return agentRadius_ * angle; // Original from paper
}

double Pathfinder::calculateEstimateGValue(const navmesh::State &state, const navmesh::State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<navmesh::State, double> &gScores) const {
  // The common edge between triangles `state` and `parentState`
  const int commonEdge = state.getEntryEdgeIndex();
  const double parentGValue = gScores.at(parentState);
  const double hValue = calculateHValue(state, goalPoint);

  // Max of {
  // 1. Distance between start and closest point on edge `commonEdge`
  double val1;
  if (state.isGoal()) {
    // Straight line from start to goal
    val1 = math::distance(startPoint, goalPoint);
  } else {
    // Straight line from start and closest point of edge to start
    val1 = distanceBetweenEdgeAndPoint(commonEdge, startPoint);
  }

  // 2. parentState.gValue + (arc around vertex shared by parentState.getEntryEdgeIndex() and `commonEdge`)
  double val2 = parentGValue;
  if (state.isGoal()) {
    // Need to add distance from entry edge to goal point
    val2 += distanceBetweenEdgeAndPoint(state.getEntryEdgeIndex(), goalPoint);
  } else if (parentState.getEntryEdgeIndex() != -1) {
    // Can calculate arc-length
    val2 += calculateArcLength(parentState.getEntryEdgeIndex(), commonEdge);
  }

  // 3. parentState.gValue + (parentState.hValue - state.hValue)
  double parentHeuristicValue;
  if (parentState.getEntryEdgeIndex() == -1) {
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

std::tuple<double, Vector, std::optional<LengthFunnel>> Pathfinder::calculateGValue(const navmesh::State &state, const navmesh::State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::map<navmesh::State, navmesh::State> &previous) const {
  // The common edge between triangles `state` and `parentState`
  const int commonEdgeIndex = state.getEntryEdgeIndex();

  if (state.isGoal()) {
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

      LengthFunnel lengthFunnel(agentRadius_);
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
  triangleCorridor.emplace_back(state.getTriangleIndex());

  if (triangleCorridor.size() == 1) {
    // Only one triangle in corridor
    result = distanceBetweenEdgeAndPoint(commonEdgeIndex, startPoint, &pointUsed);
    // std::cout << "  [G Value] (" << result << ") Straight line from start to edge " << commonEdgeIndex << std::endl; //DEBUGPRINTS
  } else {
    // Funnel from startPoint to a point that minimizes g-value
    // std::cout << "Funneling to state " << state << std::endl; //DEBUGPRINTS

    // ============================================================
    LengthFunnel lengthFunnel(agentRadius_);
    const auto commonEdge = navmesh_.getEdge(commonEdgeIndex);
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
    // std::cout << "  Funneling for g(x) from start to edge " << commonEdgeIndex << ", length: " << result << std::endl; //DEBUGPRINTS
  }
  return {result, pointUsed, optionalFunnelForCaching};
}

double Pathfinder::distanceBetweenEdgeAndPoint(int edgeIndex, const Vector &point, Vector *pointUsedForDistanceCalculation) const {
  // Return the distance between the closest end of the given edge and the given point
  const auto &[vertexA, vertexB] = navmesh_.getEdge(edgeIndex);
  return math::distanceBetweenEdgeAndPoint(vertexA, vertexB, point, pointUsedForDistanceCalculation);
}

double Pathfinder::calculateHValue(const navmesh::State &state, const Vector &goalPoint) const {
  // The h-value is the Euclidean distance between the goalPoint and the closest point to it on this edge (entry edge)
  if (state.getEntryEdgeIndex() == -1) {
    throw std::runtime_error("Logic error, calculating h value for state which has no entry edge");
  }
  if (state.isGoal()) {
    // Heuristic from the goal to the goal is 0
    return 0;
  }
  return distanceBetweenEdgeAndPoint(state.getEntryEdgeIndex(), goalPoint);
}


PathfindingAStarInfo Pathfinder::triangleAStar(const Vector &startPoint, int startTriangle, const Vector &goalPoint, int goalTriangle) const {
  PathfindingAStarInfo result;
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Trying to find shortest path from triangle " << startTriangle << " to triangle " << goalTriangle << std::endl; //DEBUGPRINTS
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  std::vector<navmesh::State> openSet; // "open set"
  std::set<navmesh::State> visited;

  std::map<navmesh::State, navmesh::State> previous;

  // Storing the best g score for each state
  std::map<navmesh::State, double> gScores;
  // Storing the best f score for each state
  std::map<navmesh::State, double> fScores;

  // Create the start state
  navmesh::State startState{startTriangle}; // There is no entry edge for the start state
  openSet.push_back(startState);

  // Initialize the result as having touched the start state
  result.trianglesDiscovered.emplace(startState.getTriangleIndex());

  // Set g score for start state
  gScores.emplace(startState, 0);
  // Calculate heuristic value for start point
  const auto distanceFromStartToGoal = math::distance(startPoint, goalPoint);
  // std::cout << "Distance from start to goal: " << distanceFromStartToGoal << std::endl;
  // Set f score for start state
  fScores.emplace(startState, distanceFromStartToGoal);

  
  double shortestPathLength = std::numeric_limits<double>::max();
  navmesh::State finalGoalState;
  bool found = false;

  while (!openSet.empty()) {
    // std::cout << "Evaluating possible next options ["; //DEBUGPRINTS
    // for (const navmesh::State &s : openSet) { //DEBUGPRINTS
      // std::cout << '{' << s << ',' << fScores.at(s) << "},"; //DEBUGPRINTS
    // } //DEBUGPRINTS
    // std::cout << ']' << std::endl; //DEBUGPRINTS

    // Find the state with minimum fScore
    auto minElementIt = std::min_element(openSet.begin(), openSet.end(), [&fScores](const navmesh::State &s1, const navmesh::State &s2) {
      // We may assume that every state is in the fScore map already
      return math::lessThan(fScores.at(s1), fScores.at(s2));
    });
    if (minElementIt == openSet.end()) {
      throw std::runtime_error("No minimum element in a nonempty vector. Logic error");
    }
    
    // Copy state, since we will delete it from the open set
    const navmesh::State currentState = *minElementIt;
    
    // std::cout << "We chose " << currentState << " as our next candidate" << std::endl; //DEBUGPRINTS
    openSet.erase(minElementIt);

    if (currentState.isGoal()) {
      // Found the goal
      result.triangleCorridor = rebuildPath(currentState, previous);
      return result;
    }

    // Look at successors
    auto successors = navmesh_.getSuccessors(currentState, goalTriangle, agentRadius_);
    for (const auto &successor : successors) {
      // std::cout << "  -Possible successor- " << successor << std::endl; //DEBUGPRINTS
      if (visited.find(successor) == visited.end()) {
        // Touched this successor
        result.trianglesDiscovered.emplace(successor.getTriangleIndex());

        // std::cout << "  -Evaluatating successor- " << successor << std::endl; //DEBUGPRINTS
        if (gScores.find(successor) == gScores.end()) {
          // No gScore yet for this state, initialize
          gScores.emplace(successor, std::numeric_limits<double>::max());
        }
        if (fScores.find(successor) == fScores.end()) {
          // No fScore yet for this state, initialize
          fScores.emplace(successor, std::numeric_limits<double>::max());
        }

        constexpr const bool kOptimalPath{true};
        if constexpr (kOptimalPath) {
          // The heuristic scale factor allows this A* algorithm to be tailored more towards:
          //  1. Djikstra's search
          //    a. A scale factor < 1 will result in a potentially wider search area
          //    b. Will trend towards a globally optimal path when the heuristic is not admissible
          //  2. Greedy Best First search
          //    a. A scale factor > 1 will result in a more direct but potentially incorrect search
          //  3. Perfect A*
          //    a. A scale factor of 1 works optimally with a consistent and admissible heuristic
          const double kHeuristicScaleFactor = 0.50;
          const auto [gValueOfSuccessor, pointUsedForGValue, optionalFunnelCreated] = calculateGValue(successor, currentState, startPoint, goalPoint, previous);
          const double oldHValue = calculateHValue(successor, goalPoint);
          const double newHValue = math::distance(pointUsedForGValue, goalPoint);
          const double hValueOfSuccessor = kHeuristicScaleFactor * std::min(oldHValue, newHValue);
          const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
          // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f (old: %.9f, new: %.9f), and fScore: %.9f\n", successor.getEntryEdgeIndex(), gValueOfSuccessor, hValueOfSuccessor, oldHValue, newHValue, fValueOfSuccessor); //DEBUGPRINTS
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
    result.trianglesSearched.emplace(currentState.getTriangleIndex());
  }

  // Open set is empty, but no path was found
  // std::cout << std::endl; //DEBUGPRINTS
  return result;
}

void Pathfinder::buildShortestPathWithinSingleTriangle(const int triangleIndex, const Vector &startPoint, const Vector &goalPoint, std::vector<std::unique_ptr<PathSegment>> &shortestPath) const {
  if (math::equal(agentRadius_, 0.0)) {
    // Don't need to worry about this, return a simple straight segment
    shortestPath.emplace_back(std::unique_ptr<StraightPathSegment>(new StraightPathSegment(startPoint, goalPoint)));
    return;
  }
  auto buildPathAvoidingVertex = [this, &startPoint, &goalPoint, &shortestPath](Vector &intersectionPoint1, Vector &intersectionPoint2, const Vector &vertex) {
    if (math::lessThan(math::distance(intersectionPoint2, startPoint), math::distance(intersectionPoint1, startPoint))) {
      // Guaranteed by the function which produces the intersection points
      throw std::runtime_error("Expecting intersectionPoint1 to be closer to the start point");
    }
    // Determine direction which we're rotating around the vertex
    double crossProductResult = math::crossProductForSign(vertex, intersectionPoint2, vertex, intersectionPoint1);
    AngleDirection direction;
    if (crossProductResult > 0) {
      direction = AngleDirection::kClockwise;
    } else if (crossProductResult < 0) {
      direction = AngleDirection::kCounterclockwise;
    } else {
      // Cross product of 0 would either mean that this isnt a valid triangle or that the agentRadius_ is 0
      throw std::runtime_error("Invalid cross-product result");
    }

    const auto segmentToCircle = math::createCircleConsciousLine(startPoint, AngleDirection::kNoDirection, vertex, direction, agentRadius_);
    const auto segmentFromCircle = math::createCircleConsciousLine(vertex, direction, goalPoint, AngleDirection::kNoDirection, agentRadius_);

    // Straight segment from start point to circle, arc around circle, straight segment from circle to goal
    shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(segmentToCircle.first, segmentToCircle.second)));
    shortestPath.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(vertex, agentRadius_, direction)));

    ArcPathSegment *arc = reinterpret_cast<ArcPathSegment*>(shortestPath.back().get());
    arc->startAngle = math::angle(vertex, segmentToCircle.second);
    arc->endAngle = math::angle(vertex, segmentFromCircle.first);
    
    shortestPath.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(segmentFromCircle.first, segmentFromCircle.second)));
  };
  const auto &[vertexA, vertexB, vertexC] = navmesh_.getTriangleVertices(triangleIndex);
  // Check if the straight path from start to goal would intersect with the circle of any of the triangle vertices
  Vector intersectionPoint1, intersectionPoint2;
  if (math::lineSegmentIntersectsWithCircle(startPoint, goalPoint, vertexA, agentRadius_, &intersectionPoint1, &intersectionPoint2) == 2) {
    buildPathAvoidingVertex(intersectionPoint1, intersectionPoint2, vertexA);
  } else if (math::lineSegmentIntersectsWithCircle(startPoint, goalPoint, vertexB, agentRadius_, &intersectionPoint1, &intersectionPoint2) == 2) {
    buildPathAvoidingVertex(intersectionPoint1, intersectionPoint2, vertexB);
  } else if (math::lineSegmentIntersectsWithCircle(startPoint, goalPoint, vertexC, agentRadius_, &intersectionPoint1, &intersectionPoint2) == 2) {
    buildPathAvoidingVertex(intersectionPoint1, intersectionPoint2, vertexC);
  } else {
    // No intersections or only single intersection points, a straight segment will do
    // We know that if there is a single intersection that one of the points of the line must be exactly on the circumference of the circle around some constraint and the other point is outside of the circle
    shortestPath.emplace_back(std::unique_ptr<StraightPathSegment>(new StraightPathSegment(startPoint, goalPoint)));
  }
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
        totalDistance += arcSegment->circleRadius * std::abs(angle);
        // std::cout << "+arc (angle start:" << arcSegment->startAngle << ", angle end: " << arcSegment->endAngle << ", angle: " << angle << ", radius:" << arcSegment->circleRadius << ") [" << totalDistance << "], "; //DEBUGPRINTS
      }
    }
  }
  // std::cout << std::endl; //DEBUGPRINTS
  return totalDistance;
}

std::vector<int> rebuildPath(navmesh::State state, const std::map<navmesh::State, navmesh::State> &previous) {
  std::vector<int> result;
  while (previous.find(state) != previous.end()) {
    if (!state.isGoal()) {
      result.emplace_back(state.getTriangleIndex());
    }
    state = previous.at(state);
  }
  result.emplace_back(state.getTriangleIndex());
  std::reverse(result.begin(), result.end());
  return result;
}

void PathfindingResult::clear() {
  shortestPath.clear();
  aStarInfo.triangleCorridor.clear();
  aStarInfo.trianglesSearched.clear();
  aStarInfo.trianglesDiscovered.clear();
}

} // namespace pathfinder
