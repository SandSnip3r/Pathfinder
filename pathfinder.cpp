#include "pathfinder.h"

#include <iostream>
#include <optional>
#include <queue>
#include <set>

float crossProduct(QPointF p1, QPointF p2, QPointF p3);
bool isPointInTriangle(QPointF pt, QPointF v1, QPointF v2, QPointF v3);
double distance(const QPointF &p1, const QPointF &p2);
std::vector<int> rebuildPath(State state, const std::map<State, State> &previous);
bool lessThan(double d1, double d2);

Pathfinder::Pathfinder(const triangleio &triangleData, const triangleio &triangleVoronoiData) : triangleData_(triangleData), triangleVoronoiData_(triangleVoronoiData) {}

std::vector<int> Pathfinder::findShortestPath(const QPointF &startPoint, const QPointF &goalPoint) {
  int startTriangle = findTriangleForPoint(startPoint);
  int goalTriangle = findTriangleForPoint(goalPoint);
  if (startTriangle == goalTriangle) {
    // Only one triangle involved
    return {startTriangle};
  }
  return triangleAStar(startPoint, startTriangle, goalPoint, goalTriangle);
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
    if (isPointInTriangle(point, vertexA, vertexB, vertexC)) {
      return triangleNumber;
    }
  }

  throw std::runtime_error("Unable to find triangle for point");
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
    val1 = distance(startPoint, goalPoint);
  } else {
    // Straight line from start and closest point of edge
    val1 = distanceBetweenEdgeAndPoint(commonEdge, startPoint);
    // TODO: I wonder if this should actually be the furthest point of the edge
  }
  
  // 2. parentState.gValue + (arc around vertex shared by parentState.entryEdge and `commonEdge`)
  // TODO: implement calculateArcLength which will use kCharacterRadius_
  //  For now, assume character radius = 0, so arc length is 0
  double val2 = parentGValue; // + calculateArcLength(commonEdge, parentState.entryEdge);
  if (state.isGoal) {
    // Need to add distance from entry edge to goal point
    val2 += distanceBetweenEdgeAndPoint(state.entryEdge, goalPoint);
  }
  
  // 3. parentState.gValue + (parentState.hValue - state.hValue)
  double val3;
  if (parentState.entryEdge == -1) {
    // This is the start
    val3 = parentGValue + (distance(startPoint, goalPoint) - hValue);
  } else {
    val3 = parentGValue + (calculateHValue(parentState, goalPoint) - hValue);
  }
  // }
  std::cout << "Potential g values for state " << state;
  printf(" are [%.9f,%.9f,%.9f]\n",val1,val2,val3);
  std::cout << std::flush;
  return std::max({val1, val2, val3});
}

double Pathfinder::distanceBetweenEdgeAndPoint(int edgeNum, const QPointF &point) const {
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
  QPointF vertexA{triangleData_.pointlist[vertexAIndex*2], triangleData_.pointlist[vertexAIndex*2+1]};
  QPointF vertexB{triangleData_.pointlist[vertexBIndex*2], triangleData_.pointlist[vertexBIndex*2+1]};
  return std::min(distance(vertexA, point), distance(vertexB, point));
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
          } else {
          }
        }
      }
    }
  }
  return result;
}

std::vector<int> Pathfinder::triangleAStar(const QPointF &startPoint, int startTriangle, const QPointF &goalPoint, int goalTriangle) const {
  std::cout << "===============================================================================" << std::endl;
  std::cout << "Trying to find shortest path from triangle " << startTriangle << " to triangle " << goalTriangle << std::endl;
  std::cout << "===============================================================================" << std::endl;
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

  // Set g score for start state
  gScores.emplace(startState, 0);
  // Calculate heuristic value for start point
  const auto distanceFromStartToGoal = distance(startPoint, goalPoint);
  std::cout << "Distance from start to goal: " << distanceFromStartToGoal << std::endl;
  // Set f score for start state
  fScores.emplace(startState, distanceFromStartToGoal);

  while (!openSet.empty()) {
    std::cout << "fScores: [";
    for (const auto &stateFScorePair : fScores) {
      std::cout << '{' << stateFScorePair.first << ':' << stateFScorePair.second << "},";
    }
    std::cout << ']' << std::endl;

    std::cout << "Evaluating possible next options [";
    for (const State &s : openSet) {
      std::cout << '{' << s << ',' << fScores.at(s) << "},";
    }
    std::cout << ']' << std::endl;

    // Find the state with minimum fScore
    auto minElementIt = std::min_element(openSet.begin(), openSet.end(), [&fScores](const State &s1, const State &s2) {
      // We may assume that every state is in the fScore map already
      return lessThan(fScores.at(s1), fScores.at(s2));
    });
    if (minElementIt == openSet.end()) {
      throw std::runtime_error("No minimum element in a nonempty vector. Logic error");
    }
    
    // Copy state, since we will delete it from the open set
    const State currentState = *minElementIt;

    if (currentState.isGoal) {
      // Done!
      std::cout << "Best option is the goal, we are done" << std::endl;
      return rebuildPath(currentState, previous);
    }

    std::cout << "We chose " << currentState << " as our next candidate" << std::endl;
    openSet.erase(minElementIt);

    // Look at successors
    auto successors = getSuccessors(currentState, goalTriangle);
    // std::cout << "Looking at neighbors of triangle " << currentState.triangleNum << ": [";
    // for (const State &state : successors) {
    //   std::cout << state << ',';
    // }
    // std::cout << ']' << std::endl;
    for (const auto &successor : successors) {
      std::cout << "  Evaluatating successor " << successor << std::endl;
      if (gScores.find(successor) == gScores.end()) {
        // No gScore yet for this state, initialize
        gScores.emplace(successor, std::numeric_limits<double>::max());
      }
      if (fScores.find(successor) == fScores.end()) {
        // No fScore yet for this state, initialize
        fScores.emplace(successor, std::numeric_limits<double>::max());
      }
      
      const double gValueOfSuccessor = calculateGValue(successor, currentState, startPoint, goalPoint, gScores);
      if (lessThan(gValueOfSuccessor, gScores.at(successor))) {
        // TODO: This comparison was suffering from a floating point issue
        // This path is better than any previous one
        if (gScores.at(successor) != std::numeric_limits<double>::max()) {
          printf("  better gScore found %.12f vs %.12f\n", gScores.at(successor), gValueOfSuccessor);
          printf("    difference: %E\n", gValueOfSuccessor-gScores.at(successor));
          std::cout << std::flush;
        }
        gScores.at(successor) = gValueOfSuccessor;

        // Update previous
        previous[successor] = currentState;

        // Update fScore
        const double hValueOfSuccessor = calculateHValue(successor, goalPoint);
        const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
        printf("  Resulting heuristic: %.9f, and fScore: %.9f\n", hValueOfSuccessor, fValueOfSuccessor);
        std::cout << std::flush;
        fScores.at(successor) = fValueOfSuccessor;

        // Add to open set
        if (std::find(openSet.begin(), openSet.end(), successor) == openSet.end()) {
          openSet.push_back(successor);
        } else {
          std::cout << "  WHOA!!!!!!!!!!!!!!! Already in open set... hmm?" << std::endl;
        }
      }
    }
  }

  // Open set is empty, but no path was found
  throw std::runtime_error("No path found");
}

/* ========================================================================== **
** ============================= Free functions ============================= **
** ========================================================================== */

float crossProduct(QPointF p1, QPointF p2, QPointF p3) {
  return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

bool isPointInTriangle(QPointF pt, QPointF v1, QPointF v2, QPointF v3) {
  bool b1, b2, b3;
  b1 = (crossProduct(pt, v1, v2) < 0.0f);
  b2 = (crossProduct(pt, v2, v3) < 0.0f);
  b3 = (crossProduct(pt, v3, v1) < 0.0f);
  return ((b1 == b2) && (b2 == b3));
}

double distance(const QPointF &p1, const QPointF &p2) {
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  return sqrt(dx*dx+dy*dy);
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

bool lessThan(double d1, double d2) {
  return (d2-d1 > 0.000001);
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