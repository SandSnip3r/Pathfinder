#include "pathfinder.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <set>
#include <string>

std::vector<int> rebuildPath(State state, const std::map<State, State> &previous);
void addToLeftOfFunnel(Funnel &funnel, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
void addToRightOfFunnel(Funnel &funnel, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString, const bool isGoal = false);
void finishFunnel(Funnel &funnel, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
bool counterclockwiseTo(const double theta, const double phi);
bool clockwiseTo(const double theta, const double phi);
double angleBetweenCenterOfCircleAndIntersectionWithTangentLine(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
double angle(const QPointF &point1, const AngleDirection point1Direction, const QPointF &point2, const AngleDirection point2Direction, const double circleRadius);
std::pair<QPointF, QPointF> intersectionsPointsOfTangentLinesToCircle(const QPointF &point, const QPointF &centerOfCircle, const double circleRadius);
void addSegmentToPath(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString);
std::pair<QPointF, QPointF> createCircleConsciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius);

PathSegment::~PathSegment() {}

Pathfinder::Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData) : triangleData_(triangleData), triangleVoronoiData_(triangleVoronoiData) {
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

  // Check if it's even possible to reach the goal segment
  if (pathCanExist(startTriangle, goalTriangle)) {

    // It is possible to reach the goal, run A* to find the shortest path
    result.aStarInfo = triangleAStar(startPoint, startTriangle, goalPoint, goalTriangle);

    if (result.aStarInfo.triangleCorridor.empty()) {
      throw std::runtime_error("A* should've found a path");
    }

    // A path was found
    std::optional<QPointF> goalPointOptional = goalPoint;
    // TODO: calculateGValue (called from triangleAStar) has already calculated this funnel
    result.shortestPath = funnel(result.aStarInfo.triangleCorridor, startPoint, goalPointOptional);
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

std::vector<std::pair<QPointF,QPointF>> Pathfinder::buildCorridor(const std::vector<int> &trianglesInCorridor) const {
  std::vector<std::pair<QPointF,QPointF>> corridorSegments;
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

std::vector<std::unique_ptr<PathSegment>> Pathfinder::funnel(const std::vector<int> &trianglesInCorridor, const QPointF &startPoint, std::optional<QPointF> &goalPoint) const {
  std::function<std::string(const QPointF&)> pointToStringFunc = [this, &startPoint, &goalPoint](const QPointF &point) -> std::string {
    if (point == startPoint) {
      return "[FUNNEL START]";
    } else if (goalPoint && point == *goalPoint) {
      return "[FUNNEL GOAL]";
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
  auto printFunnel = [&pointToStringFunc, &apexToString](const Funnel &funnel) {
    std::cout << "[";
    for (int i=0; i<funnel.size(); ++i) {
      if (i == funnel.apex_index()) {
        std::cout << apexToString(funnel.funnel_apex());
      } else {
        std::cout << pointToStringFunc(funnel.at(i));
      }
      std::cout << ',';
    }
    std::cout << "]" << std::endl;
  };

  if (trianglesInCorridor.empty()) {
    throw std::runtime_error("Trying to funnel for an empty corridor");
  }
  if (trianglesInCorridor.size() == 1) {
    if (!goalPoint) {
      throw std::runtime_error("No goal given and only one triangle!! wtf?");
    }
    // Start and goal are in the same triangle, the path is a straight line from start to goal
    std::vector<std::unique_ptr<PathSegment>> path;
    path.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(startPoint, *goalPoint)));
    return path;
  }

  // Build a list of "edges" that we'll need to pass through in this corridor
  using CorridorType = std::vector<std::pair<QPointF,QPointF>>;
  using PathType = std::vector<std::unique_ptr<PathSegment>>;
  CorridorType corridor = buildCorridor(trianglesInCorridor);
  PathType path;

  Funnel funnel(startPoint, corridor.size());

  // First and second point are the two edges of the first edge
  auto [left, right] = corridor.at(0);
  const bool areCorrectOrder = (math::crossProduct(startPoint, right, startPoint, left) > 0);
  if (!areCorrectOrder) {
    // incorrect order (left is actually right), swap
    std::swap(left, right);
  }
  // std::cout << "=============================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Starting funnel: "; //DEBUGPRINTS
  // printFunnel(funnel); //DEBUGPRINTS

  addToLeftOfFunnel(funnel, left, path, characterRadius_, pointToStringFunc);
  // std::cout << "Funnel after adding left: "; //DEBUGPRINTS
  // printFunnel(funnel); //DEBUGPRINTS

  addToRightOfFunnel(funnel, right, path, characterRadius_, pointToStringFunc);
  // std::cout << "Funnel before loop (after adding right): "; //DEBUGPRINTS
  // printFunnel(funnel); //DEBUGPRINTS

  for (int i=1; i<corridor.size(); ++i) {
    const auto [newPoint1, newPoint2] = corridor.at(i);
    if (newPoint1 == right) {
      // newPoint2 will be left
      left = newPoint2;
      addToLeftOfFunnel(funnel, left, path, characterRadius_, pointToStringFunc);
    } else if (newPoint1 == left) {
      // newPoint2 will be right
      right = newPoint2;
      addToRightOfFunnel(funnel, right, path, characterRadius_, pointToStringFunc);
    } else if (newPoint2 == right) {
      // newPoint1 will be left
      left = newPoint1;
      addToLeftOfFunnel(funnel, left, path, characterRadius_, pointToStringFunc);
    } else {
      // newPoint1 will be right
      right = newPoint1;
      addToRightOfFunnel(funnel, right, path, characterRadius_, pointToStringFunc);
    }
    // std::cout << "Funnel : "; //DEBUGPRINTS
    // printFunnel(funnel); //DEBUGPRINTS
  }

  // std::cout << "Funnel after loop : "; //DEBUGPRINTS
  // printFunnel(funnel); //DEBUGPRINTS

  QPointF goalPointToUse;
  if (!goalPoint) {
    // std::cout << "  Need to find goal for funnel" << std::endl; //DEBUGPRINTS
    // No goal given, this must mean we're funneling inside of the A* algorithm
    // We dont know where we want the goal point of the funnel to be
    //  It should be the closest point to the most recent apex
    //  However, the apex depends on the goal point
    // Any point on the left or right of the funnel could end up being the apex. We need to check all of them and use the one that results in the shortest path
    if (funnel.empty()) {
      throw std::runtime_error("Funnel is empty");
    }

    // Save the length of the current path
    const double currentPathLength = calculatePathLength(path);

    auto funnelLength = [this, &pointToStringFunc, &printFunnel](const Funnel &funnel, const std::vector<std::unique_ptr<PathSegment>> &existingPath, const QPointF &goalPoint) -> double {
      // Copy all data, since this is only a test
      auto funnelCopy = funnel;
      PathType path;
      if (funnelCopy.apex_type() != AngleDirection::kPoint) {
        // Need to copy last point of path
        ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(existingPath.back().get());
        // StraightPathSegment *straight = dynamic_cast<StraightPathSegment*>(lastPointOfExistingPath.get());
        if (arc == nullptr) {
          throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
        }
        path.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(*arc)));
      }

      // std::cout << "  ::Checking funnel length from potential apex" << std::endl; //DEBUGPRINTS
      
      if (funnelCopy.point_in_funnel(goalPoint)) {
        if (funnelCopy.front() == goalPoint) {
          // Goal is on the left, lets pop it and put it on the right
          // TODO: I dont think this logic is generic enough to catch all cases
          if (funnelCopy.front() == funnelCopy.apex_point()) {
            std::cout << "    ::Whoa! About to pop apex from left of funnel" << std::endl;
          }
          funnelCopy.pop_front();
          addToRightOfFunnel(funnelCopy, goalPoint, path, characterRadius_, pointToStringFunc, true);
        } else if (funnelCopy.back() == goalPoint) {
          // std::cout << "    ::Goal point is already in the right spot in the funnel" << std::endl; //DEBUGPRINTS
        } else {
          // std::cout << "    ::Goal point is in the funnel, but not at the right or left" << std::endl; //DEBUGPRINTS
        }
      } else {
        // Finally, add the goal to the right of the funnel
        addToRightOfFunnel(funnelCopy, goalPoint, path, characterRadius_, pointToStringFunc, true);
      }
      // And finish the algorithm, closing out the funnel
      finishFunnel(funnelCopy, path, characterRadius_, pointToStringFunc);

      // std::cout << "    ::Completed funnel : "; //DEBUGPRINTS
      // printFunnel(funnelCopy); //DEBUGPRINTS

      return calculatePathLength(path);
    };

    // Track the best option
    double shortestPathLength = std::numeric_limits<double>::max();

    // Save the final edge, we will reference it many times
    const auto [edgeStart, edgeEnd] = corridor.back();

    // For each point in the funnel, find the closest point on the target edge, then check what the overall funnel length is
    AngleDirection funnelApexAngleDirection = AngleDirection::kCounterclockwise;
    for (int funnelIndex=0; funnelIndex<funnel.size(); ++funnelIndex) {
      const QPointF &currentFunnelPoint = funnel.at(funnelIndex);
      if (currentFunnelPoint == funnel.apex_point()) {
        // We've reached the apex of the funnel, there are no more points left on the left of the funnel
        funnelApexAngleDirection = funnel.apex_type();
      }
      // Test that, if this was the final apex, would it result in the shortest path
      QPointF potentialGoal;
      if (currentFunnelPoint == edgeStart) {
        // This point of the funnel is the start of the edge, need to move it over by the character radius
        // std::cout << "  This point of the funnel (" << pointToStringFunc(currentFunnelPoint) << ") is the start of the edge. Need to move" << std::endl; //DEBUGPRINTS
        potentialGoal = edgeStart;
      } else if (currentFunnelPoint == edgeEnd) {
        // This point of the funnel is the end of the edge, need to move it over by the character radius
        // std::cout << "  This point of the funnel (" << pointToStringFunc(currentFunnelPoint) << ") is the end of the edge. Need to move" << std::endl; //DEBUGPRINTS
        potentialGoal = edgeEnd;
      } else {
        // This point of the funnel is not one of the ends of the edge
        // std::cout << "  Checking distance from a point on the funnel (" << pointToStringFunc(currentFunnelPoint) << ") to the target edge (" << pointToStringFunc(edgeStart) << "->" << pointToStringFunc(edgeEnd) << ")" << std::endl; //DEBUGPRINTS
        math::distanceBetweenEdgeAndCircleTangentIntersectionPoint(edgeStart, edgeEnd, currentFunnelPoint, characterRadius_, funnelApexAngleDirection, &potentialGoal);
      }
      bool moved = false;
      if (math::distance(edgeStart, potentialGoal) < characterRadius_) {
        // Need to move the point towards the other end of the edge
        // std::cout << "  Moving point towards edgeEnd" << std::endl; //DEBUGPRINTS
        const double edgeDx = edgeEnd.x()-edgeStart.x();
        const double edgeDy = edgeEnd.y()-edgeStart.y();
        const double edgeLength = sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
        const double ratio = characterRadius_/edgeLength;
        potentialGoal = QPointF{edgeStart.x()+edgeDx*ratio, edgeStart.y()+edgeDy*ratio};
        moved = true;
      } else if (math::distance(edgeEnd, potentialGoal) < characterRadius_) {
        // Need to move the point towards the other end of the edge
        // std::cout << "  Moving point towards edgeStart" << std::endl; //DEBUGPRINTS
        const double edgeDx = edgeStart.x()-edgeEnd.x();
        const double edgeDy = edgeStart.y()-edgeEnd.y();
        const double edgeLength = sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
        const double ratio = characterRadius_/edgeLength;
        potentialGoal = QPointF{edgeEnd.x()+edgeDx*ratio, edgeEnd.y()+edgeDy*ratio};
        moved = true;
      }
      if (moved) {
        // std::cout << "  Moved point result: " << pointToStringFunc(potentialGoal) << std::endl; //DEBUGPRINTS
      }

      // TODO: Cant we just use the existing path length up to this point and then recursively funnel from current apex to the potential goal?
      const double remainingPathLength = funnelLength(funnel, path, potentialGoal);
      const double pathLength = currentPathLength + remainingPathLength;
      if (pathLength < shortestPathLength) {
        shortestPathLength = pathLength;
        goalPointToUse = potentialGoal;
        // std::cout << "  New shortest path (len=" << shortestPathLength << ")" << std::endl; //DEBUGPRINTS
      }

      if (currentFunnelPoint == funnel.apex_point()) {
        // Now we're moving on to the right of the funnel
        funnelApexAngleDirection = AngleDirection::kClockwise;
      }
    }
    // std::cout << "Funnel ends with a segment from apex " << pointToStringFunc(funnel.apex_point()) << " to " << pointToStringFunc(goalPointToUse) << std::endl; //DEBUGPRINTS
  } else {
    goalPointToUse = *goalPoint;
    // std::cout << "  Already know goal for funnel: " << pointToStringFunc(goalPointToUse) << std::endl; //DEBUGPRINTS
  }
  // std::cout << "Adding goal (" << pointToStringFunc(goalPointToUse) << ") to funnel" << std::endl; //DEBUGPRINTS

  if (funnel.point_in_funnel(goalPointToUse)) {
    // std::cout << "Goal already in funnel" << std::endl; //DEBUGPRINTS
    if (funnel.front() == goalPointToUse) {
      // Goal is on the left, lets pop it and put it on the right
      // TODO: I dont think this logic is generic enough to catch all cases
      // std::cout << "Goal is on the left of the funnel, removing it and putting it on the right" << std::endl; //DEBUGPRINTS
      if (funnel.front() == funnel.apex_point()) {
        std::cout << "Whoa! About to pop the apex off the front of the funnel" << std::endl;
      }
      funnel.pop_front();
      addToRightOfFunnel(funnel, goalPointToUse, path, characterRadius_, pointToStringFunc, true);
    }
  } else {
    // Finally, add the goal to the right of the funnel
    // std::cout << "Goal not in funnel. Adding to the right" << std::endl; //DEBUGPRINTS
    addToRightOfFunnel(funnel, goalPointToUse, path, characterRadius_, pointToStringFunc, true);
  }
  // std::cout << "Funnel after adding goal : "; //DEBUGPRINTS
  // printFunnel(funnel); //DEBUGPRINTS
  // And finish the algorithm, closing out the funnel
  finishFunnel(funnel, path, characterRadius_, pointToStringFunc);

  // std::cout << "Final path: ["; //DEBUGPRINTS
  // for (const auto &i : path) { //DEBUGPRINTS
    // const StraightPathSegment *segment = dynamic_cast<const StraightPathSegment *>(i.get()); //DEBUGPRINTS
    // const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(i.get()); //DEBUGPRINTS
    // if (segment != nullptr) { //DEBUGPRINTS
      // std::cout << pointToStringFunc(segment->startPoint) << '-' << pointToStringFunc(segment->endPoint) << ','; //DEBUGPRINTS
    // } else if (arcSegment != nullptr) { //DEBUGPRINTS
      // std::cout << "arc " << pointToStringFunc(arcSegment->circleCenter) << "(" << arcSegment->startAngle << "->" << arcSegment->endAngle << "),"; //DEBUGPRINTS
    // } //DEBUGPRINTS
  // } //DEBUGPRINTS
  // std::cout << ']' << std::endl; //DEBUGPRINTS

  if (!goalPoint) {
    goalPoint = goalPointToUse;
  }
  return path;
}

void Pathfinder::setCharacterRadius(double value) {
  characterRadius_ = value;
}

bool Pathfinder::pointIsInTriangle(const QPointF &point, const int triangleNum) const {
  const int vertexIndexA = triangleData_.trianglelist[triangleNum*3];
  const int vertexIndexB = triangleData_.trianglelist[triangleNum*3+1];
  const int vertexIndexC = triangleData_.trianglelist[triangleNum*3+2];
  if (vertexIndexA >= triangleData_.numberofpoints ||
      vertexIndexB >= triangleData_.numberofpoints ||
      vertexIndexC >= triangleData_.numberofpoints) {
    throw std::runtime_error("Triangle references vertex which does not exist");
  }
  const QPointF vertexA{triangleData_.pointlist[vertexIndexA*2], triangleData_.pointlist[vertexIndexA*2+1]};
  const QPointF vertexB{triangleData_.pointlist[vertexIndexB*2], triangleData_.pointlist[vertexIndexB*2+1]};
  const QPointF vertexC{triangleData_.pointlist[vertexIndexC*2], triangleData_.pointlist[vertexIndexC*2+1]};

  // Triangles' vertices are listed in CCW order (might matter for checking if a point lies within a triangle)
  return math::isPointInTriangle(point, vertexA, vertexB, vertexC);
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
  
  const QPointF edge1VertexA{triangleData_.pointlist[edge1VertexAIndex*2], triangleData_.pointlist[edge1VertexAIndex*2+1]};
  const QPointF edge1VertexB{triangleData_.pointlist[edge1VertexBIndex*2], triangleData_.pointlist[edge1VertexBIndex*2+1]};
  const QPointF edge2VertexA{triangleData_.pointlist[edge2VertexAIndex*2], triangleData_.pointlist[edge2VertexAIndex*2+1]};
  const QPointF edge2VertexB{triangleData_.pointlist[edge2VertexBIndex*2], triangleData_.pointlist[edge2VertexBIndex*2+1]};
  const double angle = math::angleBetweenVectors(edge1VertexA, edge1VertexB, edge2VertexA, edge2VertexB);
  // TODO: Sometimes this arclength causes weird paths
  // return 0;
  // return 0.1 * characterRadius_ * angle;
  return characterRadius_ * angle; // Original from paper
}

std::pair<double, QPointF> Pathfinder::calculateGValue(const State &state, const State &parentState, const QPointF &startPoint, const QPointF &goalPoint, const std::map<State, State> &previous) const {
  // The common edge between triangles `state` and `parentState`
  const int commonEdge = state.entryEdge;

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
      std::optional<QPointF> goalPointOptional = goalPoint;
      // std::cout << "Funneling to state " << state << std::endl; //DEBUGPRINTS
      std::vector<std::unique_ptr<PathSegment>> shortestPath = funnel(triangleCorridor, startPoint, goalPointOptional);
      result = calculatePathLength(shortestPath);
      // std::cout << "  [G Value] (" << result << "), " << shortestPath.size() << "-segment path from start to goal" << std::endl; //DEBUGPRINTS
    }
    return {result, goalPoint};
  }

  // State is not the goal
  double result;
  QPointF pointUsed;
  std::vector<int> triangleCorridor = rebuildPath(parentState, previous);
  // Add the current triangle to the corridor
  triangleCorridor.emplace_back(state.triangleNum);
  
  if (triangleCorridor.size() == 1) {
    // Only one triangle in corridor
    result = distanceBetweenEdgeAndPoint(commonEdge, startPoint, &pointUsed);
    // std::cout << "  [G Value] (" << result << ") Straight line from start to edge " << commonEdge << std::endl; //DEBUGPRINTS
  } else {
    // Funnel from startPoint to a point that minimizes g-value
    std::optional<QPointF> pointUsedResult;
    // std::cout << "Funneling to state " << state << std::endl; //DEBUGPRINTS
    // TODO: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // We want to have the result of this funnel so we can cache it
    //  In fact, this will (probably?) end up being the result of the pathfinding algorithm
    // Using the cached funnel, there might be an apex that is not the startPoint
    //  In this case, the funnel we build will be smaller.
    // We also need to save the pathlength of that cached funnel up to that apex
    // TODO: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    std::vector<std::unique_ptr<PathSegment>> shortestPath = funnel(triangleCorridor, startPoint, pointUsedResult); // No goal given (empty optional)
    if (!pointUsedResult) {
      throw std::runtime_error("Dont know point used in funnel");
    }
    pointUsed = *pointUsedResult;
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
    result = calculatePathLength(shortestPath);
    // std::cout << "  Funneling for g(x) from start to edge " << commonEdge << ", length: " << result << std::endl; //DEBUGPRINTS
  }
  return {result, pointUsed};
}

QPointF Pathfinder::midpointOfEdge(int edgeNum) const {
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
  const auto dx = vertexB.x()-vertexA.x();
  const auto dy = vertexB.y()-vertexA.y();
  return QPointF{vertexA.x()+dx/2, vertexB.y()+dy/2};
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

  return math::distanceBetweenEdgeAndPoint(vertexA, vertexB, point, pointUsedForDistanceCalculation);
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


PathfindingAStarInfo Pathfinder::triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const {
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
        
        const auto gValueResult = calculateGValue(successor, currentState, startPoint, goalPoint, previous);
        const double gValueOfSuccessor = gValueResult.first;
        const QPointF pointUsedForGValue = gValueResult.second;
        const double oldHValue = calculateHValue(successor, goalPoint);
        const double newHValue = math::distance(pointUsedForGValue, goalPoint);
        const double hValueOfSuccessor = std::min(oldHValue, newHValue);
        const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
        // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f (old: %.9f, new: %.9f), and fScore: %.9f\n", successor.entryEdge, gValueOfSuccessor, hValueOfSuccessor, oldHValue, newHValue, fValueOfSuccessor); //DEBUGPRINTS
        if (math::lessThan(fValueOfSuccessor, fScores.at(successor))) {
          // TODO: This comparison was suffering from a floating point issue

          // std::cout << "    Better fscore" << std::endl; //DEBUGPRINTS
          // Update previous
          previous[successor] = currentState;

          // Update fScore
          fScores.at(successor) = fValueOfSuccessor;

          // Add to open set if not already in
          if (std::find(openSet.begin(), openSet.end(), successor) == openSet.end()) {
            openSet.push_back(successor);
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
  // std::cout << std::endl; // TODO: Remove //DEBUGPRINTS
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
      double angle = math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, (arcSegment->angleDirection == AngleDirection::kCounterclockwise));
      totalDistance += arcSegment->circleRadius * (angle < 0 ? -angle : angle);
      // std::cout << "+arc (angle start:" << arcSegment->startAngle << ", angle end: " << arcSegment->endAngle << ", angle: " << angle << ", radius:" << arcSegment->circleRadius << ") [" << totalDistance << "], "; //DEBUGPRINTS
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
  // std::cout << "  ~Adding segment to path" << std::endl; //DEBUGPRINTS
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
    // std::cout << "    ~Finishing arc (angle: " << arc->endAngle << ")" << std::endl; //DEBUGPRINTS
  }

  // Second, add a straight segment between the apexes
  // std::cout << "    ~Adding straight segment from " << pointToString(edge.first) << " to " << pointToString(edge.second) << std::endl; //DEBUGPRINTS
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
    // std::cout << "    ~Not the end, adding the start of another arc (angle: " << arc->startAngle << ")" << std::endl; //DEBUGPRINTS
  }
}

std::pair<QPointF, QPointF> createCircleConsciousLine(const QPointF &point1, const AngleDirection &point1Direction, const QPointF &point2, const AngleDirection &point2Direction, const double circleRadius) {
  if (circleRadius < 0.001) {
    return {point1, point2};
  }
  QPointF lineStart, lineEnd;
  
  if (point1Direction == AngleDirection::kPoint) {
    lineStart = point1;
  } else {
    // point1 is a circle
    // Does point2 lie within the first circle?
    const auto distanceBetweenPoints = math::distance(point1, point2);
    if (math::lessThan(distanceBetweenPoints, circleRadius)) {
      // TODO: point2 is inside point1's circle. Handle
      // std::cout << "point2 is inside point1's circle" << std::endl;
    } else if (math::equal(distanceBetweenPoints, circleRadius)) {
      // point2 on the circumference of point1's circle
      // the line is really not a line
      return {point2, point2};
    }
  }
  if (point2Direction == AngleDirection::kPoint) {
    lineEnd = point2;
  } else {
    // point2 is a circle
    // Does point1 lie within the first circle?
    const auto distanceBetweenPoints = math::distance(point1, point2);
    if (math::lessThan(distanceBetweenPoints, circleRadius)) {
      // TODO: point1 is inside point2's circle. Handle
      // std::cout << "point1 is inside point2's circle" << std::endl;
    } else if (math::equal(distanceBetweenPoints, circleRadius)) {
      // point1 on the circumference of point2's circle
      // the line is really not a line
      return {point1, point1};
    }
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

void addToLeftOfFunnel(Funnel &funnel, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString) {
  auto newWedgeIsClockwiseToLeftmostWedge = [&funnel, &point, &circleRadius]() {
    if (funnel.size() < 2) {
      throw std::runtime_error("This function requires a funnel with at least two items");
    }
    if (funnel.front() == funnel.apex_point()) {
      throw std::runtime_error("Assumption that the front of the funnel is not the apex was broken");
    }

    const QPointF leftmostEdgePoint1 = funnel.at(1);
    const QPointF leftmostEdgePoint2 = funnel.front();
    AngleDirection leftmostEdgePoint1Direction;
    const AngleDirection leftmostEdgePoint2Direction = AngleDirection::kCounterclockwise;

    // We know that funnel.front() is not the apex, but funnel.at(1) could be
    if (funnel.at(1) == funnel.apex_point()) {
      leftmostEdgePoint1Direction = funnel.apex_type();
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
  // std::cout << "  >> Adding " << pointToString(point) << " to left of funnel" << std::endl; //DEBUGPRINTS

  std::optional<Apex> newApex;
  // Make sure there is at least one edge in the funnel
  if (funnel.size() >= 2) {
    // Remove edges that are counterclockwise to this new potential edges
    while (funnel.front() != funnel.apex_point() && newWedgeIsClockwiseToLeftmostWedge()) {
      // std::cout << "    >> Popping from left of funnel" << std::endl; //DEBUGPRINTS
      funnel.pop_front();
    }

    // Need to check if this new left edge would cross over the apex
    while (funnel.front() == funnel.apex_point() && funnel.size() >= 2) {
      std::pair<QPointF, QPointF> newWedge = createCircleConsciousLine(funnel.front(), funnel.apex_type(), point, AngleDirection::kCounterclockwise, circleRadius);
      std::pair<QPointF, QPointF> firstRightEdge = createCircleConsciousLine(funnel.front(), funnel.apex_type(), funnel.at(1), AngleDirection::kClockwise, circleRadius);
      if (math::crossProduct(newWedge.first, newWedge.second, firstRightEdge.first, firstRightEdge.second) > 0) {
        // New point crosses over apex
        // std::cout << "    >> New point crosses over apex" << std::endl; //DEBUGPRINTS
        if (math::distance(newWedge.first, newWedge.second) < math::distance(firstRightEdge.first, firstRightEdge.second)) {
          // New point is closer and should instead be the apex
          // std::cout << "      >> New point is closer and should instead be the apex" << std::endl; //DEBUGPRINTS
          newApex = Apex{point, AngleDirection::kCounterclockwise};
          
          addSegmentToPath(funnel.funnel_apex(), newWedge, *newApex, path, circleRadius, pointToString);

          // Remove old apex
          funnel.pop_front();

          // Apex is now a point not yet in the funnel, this loop is done
          break;
        } else {
          // Add the apex to the path
          // std::cout << "      >> Normal add of apex to path" << std::endl; //DEBUGPRINTS
          Apex updatedApex{funnel.at(1), AngleDirection::kClockwise};

          addSegmentToPath(funnel.funnel_apex(), firstRightEdge, updatedApex, path, circleRadius, pointToString);

          // Remove old apex
          funnel.pop_front();

          // Front is now the current apex, update
          funnel.set_apex_index(0);
          funnel.set_apex_type(updatedApex.apexType);
        }
      } else {
        // Done
        // std::cout << "    >> Done" << std::endl; //DEBUGPRINTS
        break;
      }
    }
  }
  // Finally, add point to left of funnel
  if (newApex) {
    // Add new apex to funnel
    funnel.push_front_apex(*newApex);
  } else {
    // Only adding normal point to funnel
    funnel.push_front(point);
  }
}

void addToRightOfFunnel(Funnel &funnel, const QPointF &point, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString, const bool isGoal) {
  auto newWedgeIsCounterclockwiseToRightmostWedge = [&funnel, &point, &circleRadius, &isGoal]() {
    if (funnel.size() < 2) {
      throw std::runtime_error("This function requires a funnel with at least two items");
    }
    if (funnel.back() == funnel.apex_point()) {
      throw std::runtime_error("Assumption that the back of the funnel is not the apex was broken");
    }

    const QPointF rightmostEdgePoint1 = funnel.at(funnel.size()-2);
    const QPointF rightmostEdgePoint2 = funnel.back();
    AngleDirection rightmostEdgePoint1Direction;
    const AngleDirection rightmostEdgePoint2Direction = AngleDirection::kClockwise;

    // We know that funnel.back() is not the apex, but funnel.at(funnel.size()-2) could be
    if (funnel.at(funnel.size()-2) == funnel.apex_point()) {
      rightmostEdgePoint1Direction = funnel.apex_type();
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
  // std::cout << "  << Adding " << pointToString(point) << " to right of funnel" << std::endl; //DEBUGPRINTS

  std::optional<Apex> newApex;
  // Make sure there is at least one edge in the funnel
  if (funnel.size() >= 2) {
    // Remove edges that are clockwise to this new potential edges
    while (funnel.back() != funnel.apex_point() && newWedgeIsCounterclockwiseToRightmostWedge()) {
      // std::cout << "    << Popping from right of funnel" << std::endl; //DEBUGPRINTS
      funnel.pop_back();
    }

    // Need to check if this new right edge would cross over the apex
    while (funnel.back() == funnel.apex_point() && funnel.size() >= 2) {
      // TODO: This apex check is dangerous because we modify what the apex should be in this loop, but we dont change it in the 'funnel'
      std::pair<QPointF, QPointF> newWedge = createCircleConsciousLine(funnel.back(), funnel.apex_type(), point, (isGoal ? AngleDirection::kPoint : AngleDirection::kClockwise), circleRadius);
      std::pair<QPointF, QPointF> firstLeftEdge = createCircleConsciousLine(funnel.back(), funnel.apex_type(), funnel.at(funnel.size()-2), AngleDirection::kCounterclockwise, circleRadius);
      if (math::crossProduct(newWedge.first, newWedge.second, firstLeftEdge.first, firstLeftEdge.second) <= 0) {
        // New point crosses over apex
        // std::cout << "    << New point crosses over apex" << std::endl; //DEBUGPRINTS
        if (math::distance(newWedge.first, newWedge.second) < math::distance(firstLeftEdge.first, firstLeftEdge.second)) {
          // New point is closer and should instead be the apex
          // std::cout << "      << New point is closer and should instead be the apex" << std::endl; //DEBUGPRINTS

          newApex = Apex{point, (isGoal ? AngleDirection::kPoint : AngleDirection::kClockwise)};
          
          addSegmentToPath(funnel.funnel_apex(), newWedge, *newApex, path, circleRadius, pointToString);

          // Remove old apex
          funnel.pop_back();

          // Apex is now a point not yet in the funnel, this loop is done
          break;
        } else {
          // std::cout << "      << Normal add of apex to path" << std::endl; //DEBUGPRINTS
          // Add the apex to the path
          Apex updatedApex{funnel.at(funnel.size()-2), AngleDirection::kCounterclockwise};

          addSegmentToPath(funnel.funnel_apex(), firstLeftEdge, updatedApex, path, circleRadius, pointToString);

          // Remove old apex
          funnel.pop_back();

          // Back is now the current apex, update
          funnel.set_apex_index(funnel.size()-1);
          funnel.set_apex_type(updatedApex.apexType);
        }
      } else {
        // Done
        // std::cout << "    << Done" << std::endl; //DEBUGPRINTS
        break;
      }
    }
  }
  // Finally, add point to right of funnel
  if (newApex) {
    // Add new apex to funnel
    // std::cout << "  << Adding point to right of funnel, as apex" << std::endl; //DEBUGPRINTS
    funnel.push_back_apex(*newApex);
  } else {
    // Only adding normal point to funnel
    // std::cout << "  << Adding point to right of funnel" << std::endl; //DEBUGPRINTS
    funnel.push_back(point);
  }
}

void finishFunnel(Funnel &funnel, std::vector<std::unique_ptr<PathSegment>> &path, const double circleRadius, std::function<std::string(const QPointF&)> pointToString) {
  AngleDirection t1 = funnel.apex_type();
  AngleDirection t2 = AngleDirection::kClockwise;
  for (int i=funnel.apex_index(); i<funnel.size()-1;) {
    if (i+1 == funnel.size()-1) {
      // Next point is the end of the last right edge
      t2 = AngleDirection::kPoint;
    }
    std::pair<QPointF, QPointF> newEdge = createCircleConsciousLine(funnel.at(i), t1, funnel.at(i+1), t2, circleRadius);
    addSegmentToPath(Apex{funnel.at(i), t1}, newEdge, Apex{funnel.at(i+1), t2}, path, circleRadius, pointToString);
    t1 = AngleDirection::kClockwise;
    i += 1;
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