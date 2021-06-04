template<typename NavmeshType>
Pathfinder<NavmeshType>::Pathfinder(const NavmeshType &navmesh, const double agentRadius) : navmesh_(navmesh), agentRadius_(agentRadius) {
  // Initialize debug logger
  DebugLogger::instance().setPointToIndexFunction(std::bind(&NavmeshType::getVertexIndex, std::cref(navmesh_), std::placeholders::_1));
}

template<typename NavmeshType>
template<typename PointType>
std::pair<Vector, typename Pathfinder<NavmeshType>::IndexType> Pathfinder<NavmeshType>::getPointAwayFromConstraint(const PointType &point) const {
  Vector point2d = NavmeshType::to2dPoint(point);
  std::optional<IndexType> triangleForPoint = navmesh_.findTriangleForPoint(point2d);
  if (!triangleForPoint) {
    throw std::runtime_error("Unable to find valid triangle for point");
  }
  constexpr const int kMaxIterations{5};
  int iterationCount{0};
  std::optional<Vector> collidingConstraint = getCollidingConstraint(point2d, *triangleForPoint);
  while (collidingConstraint.has_value()) {
    if (iterationCount == kMaxIterations) {
      throw std::runtime_error("Cannot point away from constraint");
    }

    // Push point out to radius of circle.
    point2d = math::extendLineSegmentToLength(*collidingConstraint, point2d, agentRadius_*1.01);
    triangleForPoint = navmesh_.findTriangleForPoint(point2d);
    if (!triangleForPoint) {
      throw std::runtime_error("Unable to find valid triangle for point");
    }

    // Check if the new point overlaps with any constraint.
    collidingConstraint = getCollidingConstraint(point2d, *triangleForPoint);

    // Keep track that we dont get stuck here for too long.
    ++iterationCount;
  }
  return {point2d, *triangleForPoint};
}

template<typename NavmeshType>
template<typename PointType>
typename Pathfinder<NavmeshType>::PathfindingResult Pathfinder<NavmeshType>::findShortestPath(const PointType &startPoint, const PointType &goalPoint) const {
  lengthFunnelCache_.clear();
  const auto [startPoint2d, startTriangle] = getPointAwayFromConstraint(startPoint);
  const auto [goalPoint2d, goalTriangle] = getPointAwayFromConstraint(goalPoint);

  const auto startState = navmesh_.createStartState(startPoint, startTriangle);
  const auto goalState = navmesh_.createGoalState(goalPoint, goalTriangle);

  PathfindingResult result;
  {
    auto goalStateButNotIsGoal = goalState;
    goalStateButNotIsGoal.setIsGoal(false);
    if (startState == goalStateButNotIsGoal) {
      // Only one state/triangle involved
      buildShortestPathWithinSingleTriangle(startTriangle, startPoint2d, goalPoint2d, result.shortestPath);
      // Only one state/triangle in corridor
      result.aStarInfo.triangleCorridor.emplace_back(startTriangle);
      // Only one state/triangle looked at
      result.aStarInfo.trianglesDiscovered.emplace(startTriangle);
      // Only one state/triangle evaluated completely
      result.aStarInfo.trianglesSearched.emplace(startTriangle);
      return result;
    }
  }

  // TODO: Could do a quick check to see if a path is even possible
  // Run A* to find the shortest path
  result.aStarInfo = triangleAStar(startPoint2d, startState, goalPoint2d, goalState);
  if (!result.aStarInfo.triangleCorridor.empty()) {
    // A path was found
    const auto corridor = buildCorridor(result.aStarInfo.triangleCorridor);
    PathFunnel pathFunnel(agentRadius_);
    pathFunnel.funnelWithGoal(corridor, startPoint2d, goalPoint2d);
    result.shortestPath = pathFunnel.getPath();
  }

  // In the case that no path exists, the result is empty
  return result;
}

template<typename NavmeshType>
std::optional<Vector> Pathfinder<NavmeshType>::getCollidingConstraint(const Vector &point, const IndexType triangleIndex) const {
  const auto verticesIndices = navmesh_.getTriangleVertexIndices(triangleIndex);

  for (const auto vertexIndex : {std::get<0>(verticesIndices), std::get<1>(verticesIndices), std::get<2>(verticesIndices)}) {
    const auto marker = navmesh_.getVertexMarker(vertexIndex);
    if (marker != 0) {
      // Constraint vertex
      const auto vertex = navmesh_.getVertex(vertexIndex);
      if (math::lessThan(math::distance(point, vertex), agentRadius_)) {
        // Point is too close to vertex
        return vertex;
      }
    }
  }
  return {};
}

template<typename NavmeshType>
void Pathfinder<NavmeshType>::buildShortestPathWithinSingleTriangle(const IndexType triangleIndex, const Vector &startPoint, const Vector &goalPoint, std::vector<std::unique_ptr<PathSegment>> &shortestPath) const {
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
  const auto [vertexA, vertexB, vertexC] = navmesh_.getTriangleVertices(triangleIndex);
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

template<typename NavmeshType>
double Pathfinder<NavmeshType>::distanceBetweenEdgeAndPoint(IndexType edgeIndex, const Vector &point, Vector *pointUsedForDistanceCalculation) const {
  // Return the distance between the closest end of the given edge and the given point
  const auto [vertexA, vertexB] = navmesh_.getEdge(edgeIndex);
  return math::distanceBetweenEdgeAndPoint(vertexA, vertexB, point, pointUsedForDistanceCalculation);
}

template<typename NavmeshType>
double Pathfinder<NavmeshType>::calculateArcLength(const IndexType edge1Index, const IndexType edge2Index) const {
  // Return the length of the arc from one edge to another (they share a vertex)
  const auto [edge1Vertex1, edge1Vertex2] = navmesh_.getEdge(edge1Index);
  const auto [edge2Vertex1, edge2Vertex2] = navmesh_.getEdge(edge1Index);
  const double angle = math::angleBetweenVectors(edge1Vertex1, edge1Vertex2, edge2Vertex1, edge2Vertex2);
  // TODO: Sometimes this arclength causes weird paths
  // return 0;
  // return 0.1 * agentRadius_ * angle;
  return agentRadius_ * angle; // Original from paper
}

template<typename NavmeshType>
double Pathfinder<NavmeshType>::calculateHValue(const State &state, const Vector &goalPoint) const {
  // The h-value is the Euclidean distance between the goalPoint and the closest point to it on this edge (entry edge)
  if (!state.hasEntryEdgeIndex()) {
    throw std::runtime_error("Logic error, calculating h value for state which has no entry edge");
  }
  if (state.isGoal()) {
    // Heuristic from the goal to the goal is 0
    return 0;
  }
  return distanceBetweenEdgeAndPoint(state.getEntryEdgeIndex(), goalPoint);
}

template<typename NavmeshType>
double Pathfinder<NavmeshType>::calculateEstimateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, double> &gScores) const {
  // The common edge between triangles `state` and `parentState`
  const auto commonEdgeIndex = state.getEntryEdgeIndex();
  const double parentGValue = gScores.at(parentState);
  const double hValue = calculateHValue(state, goalPoint);

  // Max of {
  // 1. Distance between start and closest point on edge `commonEdgeIndex`
  double val1;
  if (state.isGoal()) {
    // Straight line from start to goal
    val1 = math::distance(startPoint, goalPoint);
  } else {
    // Straight line from start and closest point of edge to start
    val1 = distanceBetweenEdgeAndPoint(commonEdgeIndex, startPoint);
  }

  // 2. parentState.gValue + (arc around vertex shared by parentState.getEntryEdgeIndex() and `commonEdgeIndex`)
  double val2 = parentGValue;
  if (state.isGoal()) {
    // Need to add distance from entry edge to goal point
    val2 += distanceBetweenEdgeAndPoint(state.getEntryEdgeIndex(), goalPoint);
  } else if (parentState.hasEntryEdgeIndex()) {
    // Can calculate arc-length
    val2 += calculateArcLength(parentState.getEntryEdgeIndex(), commonEdgeIndex);
  }

  // 3. parentState.gValue + (parentState.hValue - state.hValue)
  double parentHeuristicValue;
  if (!parentState.hasEntryEdgeIndex()) {
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

template<typename NavmeshType>
std::tuple<double, Vector, std::optional<LengthFunnel>> Pathfinder<NavmeshType>::calculateGValue(const State &state, const State &parentState, const Vector &startPoint, const Vector &goalPoint, const std::unordered_map<State, State> &previous) const {
  // The common edge between triangles `state` and `parentState`
  const auto commonEdgeIndex = state.getEntryEdgeIndex();

  if (state.isGoal()) {
    // Find length of actual path from start to goal
    double result;
    auto triangleCorridor = rebuildPath(parentState, previous);
    
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
  auto triangleCorridor = rebuildPath(parentState, previous);
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

      // std::cout << "We have a cached funnel: "; //DEBUGPRINTS
      // DebugLogger::instance().printFunnel(lengthFunnel.getFunnel()); //DEBUGPRINTS
      // std::cout << "Which was built from the following corridor: [ "; //DEBUGPRINTS
      // for (auto i : triangleCorridor) { //DEBUGPRINTS
        // std::cout << '(' << (i>>16) << ',' << (i&0xFFFF) << ") "; //DEBUGPRINTS
      // } //DEBUGPRINTS
      // std::cout << "]" << std::endl; //DEBUGPRINTS

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
    // std::cout << "  Funneling for g(x) from start to edge " << (commonEdgeIndex&0xFFFF) << ", length: " << result << std::endl; //DEBUGPRINTS
  }
  return {result, pointUsed, optionalFunnelForCaching};
}

template<typename NavmeshType>
typename Pathfinder<NavmeshType>::PathfindingAStarInfo Pathfinder<NavmeshType>::triangleAStar(const Vector &startPoint, const State &startState, const Vector &goalPoint, const State &goalState) const {
  PathfindingAStarInfo result;
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Trying to find shortest path from " << startState << " to " << goalState << std::endl; //DEBUGPRINTS
  // std::cout << "===============================================================================" << std::endl; //DEBUGPRINTS
  struct StateAndWeightPtr {
    StateAndWeightPtr(State s, double &w) : state(s), weightPtr(&w) {}
    explicit operator State() const { return state; }
    State state;
    double *weightPtr;
  };
  struct Comp {
    bool operator()(const StateAndWeightPtr& a, const StateAndWeightPtr& b) const {
      return *a.weightPtr > *b.weightPtr;
    }
  };
  BinaryMinHeap<State,StateAndWeightPtr,Comp> openSetMinHeap;
  std::unordered_set<State> visited;

  // Keep track of back-pointers so we can rebuild our path
  std::unordered_map<State, State> previous;

  // Storing the best g score for each state
  std::unordered_map<State, double> gScores;
  // Storing the best f score for each state
  std::unordered_map<State, double> fScores;

  // Set g score for start state
  gScores.emplace(startState, 0);
  // Calculate heuristic value for start point
  const auto distanceFromStartToGoal = math::distance(startPoint, goalPoint);
  // Set f score for start state
  auto startStateFScoreEmplaceResult = fScores.emplace(startState, distanceFromStartToGoal);
  if (!startStateFScoreEmplaceResult.second) {
    throw std::runtime_error("Uanble to insert fscore for start state");
  }

  // Start with the start state
  openSetMinHeap.push_back(startState, {startState, startStateFScoreEmplaceResult.first->second});
  openSetMinHeap.bubbleUp(startState);

  // Initialize the result as having touched the start state
  result.trianglesDiscovered.emplace(startState.getTriangleIndex());

  while (!openSetMinHeap.empty()) {
    // Find the state with minimum fScore
    const auto currentState = openSetMinHeap.top().state;
    openSetMinHeap.pop();
    // std::cout << "We chose " << currentState << " as our next candidate" << std::endl; //DEBUGPRINTS

    if (currentState.isGoal()) {
      // Found the goal
      result.triangleCorridor = rebuildPath(currentState, previous);
      return result;
    }

    // Look at successors
    auto successors = navmesh_.getSuccessors(currentState, goalState, agentRadius_);
    for (const auto &successor : successors) {
      // std::cout << "  -Possible successor- " << successor << std::endl; //DEBUGPRINTS
      if (visited.find(successor) == visited.end()) {
        // Track that we discovered this successor
        result.trianglesDiscovered.emplace(successor.getTriangleIndex());

        // std::cout << "  -Evaluatating successor- " << successor << std::endl; //DEBUGPRINTS
        if (gScores.find(successor) == gScores.end()) {
          // No gScore yet for this state, initialize
          gScores.emplace(successor, std::numeric_limits<double>::max());
        }
        auto fScoreIt = fScores.find(successor);
        if (fScoreIt == fScores.end()) {
          // No fScore yet for this state, initialize
          auto res = fScores.emplace(successor, std::numeric_limits<double>::max());
          if (!res.second) {
            throw std::runtime_error("Unable to insert f score for state");
          }
          fScoreIt = res.first;
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
          const double kHeuristicScaleFactor = 1;
          const auto [gValueOfSuccessor, pointUsedForGValue, optionalFunnelCreated] = calculateGValue(successor, currentState, startPoint, goalPoint, previous);
          const double oldHValue = calculateHValue(successor, goalPoint);
          const double newHValue = math::distance(pointUsedForGValue, goalPoint);
          const double hValueOfSuccessor = kHeuristicScaleFactor * std::min(oldHValue, newHValue);
          const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
          // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f (old: %.9f, new: %.9f), and fScore: %.9f\n", successor.getEntryEdgeIndex(), gValueOfSuccessor, hValueOfSuccessor, oldHValue, newHValue, fValueOfSuccessor); //DEBUGPRINTS
          if (math::lessThan(fValueOfSuccessor, fScoreIt->second)) {

            if (fScoreIt->second != std::numeric_limits<double>::max()) {
              // std::cout << "    Better fscore" << std::endl; //DEBUGPRINTS
            }
            // Update previous
            auto prevIt = previous.find(successor);
            if (prevIt != previous.end()) {
              prevIt->second = currentState;
            } else {
              previous.emplace(successor, currentState);
            }

            if (optionalFunnelCreated) {
              // Funnel was created, cache it
              // May overwrite existing cached funnel, that's intentional
              if (fScoreIt->second != std::numeric_limits<double>::max()) {
                // std::cout << "    Found a better fscore for state " << successor << ", overwriting cached funnel" << std::endl; //DEBUGPRINTS
              } else {
                // std::cout << "    Caching first funnel for state " << successor << std::endl; //DEBUGPRINTS
              }
              lengthFunnelCache_[successor] = *optionalFunnelCreated;
            }

            // Update fScore
            fScoreIt->second = fValueOfSuccessor;

            // Add to open set if not already in
            if (!openSetMinHeap.contains(successor)) {
              openSetMinHeap.push_back(successor, {successor, fScoreIt->second});
            }

            // Sucessor is guaranteed to be in the minheap now, bubble it up to the right position
            if (!openSetMinHeap.contains(successor)) {
              throw std::runtime_error("Min heap does not contain state");
            }
            openSetMinHeap.bubbleUp(successor);
          }
        } else {
          const double gValueOfSuccessor = calculateEstimateGValue(successor, currentState, startPoint, goalPoint, gScores);
          const double hValueOfSuccessor = calculateHValue(successor, goalPoint);
          const double fValueOfSuccessor = gValueOfSuccessor + hValueOfSuccessor;
          // printf("    A* g(x) of successor across edge %d is %.9f. Resulting heuristic: %.9f, and fScore: %.9f\n", successor.entryEdge, gValueOfSuccessor, hValueOfSuccessor, fValueOfSuccessor); //DEBUGPRINTS
          if (math::lessThan(fValueOfSuccessor, fScoreIt->second)) {
            // std::cout << "    Better fscore" << std::endl; //DEBUGPRINTS
            // Update previous
            auto prevIt = previous.find(successor);
            if (prevIt != previous.end()) {
              prevIt->second = currentState;
            } else {
              previous.emplace(successor, currentState);
            }

            // Update gScore
            gScores.at(successor) = gValueOfSuccessor;

            // Update fScore
            fScoreIt->second = fValueOfSuccessor;

            // Add to open set if not already in
            if (!openSetMinHeap.contains(successor)) {
              openSetMinHeap.push_back(successor, {successor, fScoreIt->second});
            }

            // Sucessor is guaranteed to be in the minheap now, bubble it up to the right position
            if (!openSetMinHeap.contains(successor)) {
              throw std::runtime_error("Min heap does not contain state");
            }
            openSetMinHeap.bubbleUp(successor);
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

template<typename NavmeshType>
bool Pathfinder<NavmeshType>::pathCanExist(const State &startState, const State &goalState) const {
  // Use breadth-first search to try to quickly find if a path exists between these two triangles

  std::set<State> visitedStates;
  // Use a set for a quicker lookup to check if we've already queued a State rather than overfilling the queue
  std::set<State> alreadyQueuedStates;
  std::queue<State> stateQueue;

  stateQueue.push(startState);
  alreadyQueuedStates.insert(startState);

  while (!stateQueue.empty()) {
    State currentState = stateQueue.front();
    stateQueue.pop();

    const auto successors = navmesh_.getSuccessors(currentState, goalState, agentRadius_);
    for (const auto &successorState : successors) {
      if (successorState.isSameTriangleAs(goalState)) {
        // Found a path to the goal
        return true;
      }
      if (visitedStates.find(successorState) == visitedStates.end()) {
        // Only care about states that we havent visited
        if (alreadyQueuedStates.find(successorState) == alreadyQueuedStates.end()) {
          // Not yet queued, queue it
          stateQueue.push(successorState);
          alreadyQueuedStates.insert(successorState);
        }
      }
    }
    // Mark current state as visited
    visitedStates.insert(currentState);
  }

  // Never found the goal triangle, must be impossible to reach
  return false;
}

template<typename NavmeshType>
std::vector<std::pair<Vector,Vector>> Pathfinder<NavmeshType>::buildCorridor(const std::vector<IndexType> &trianglesInCorridor) const {
  std::vector<std::pair<Vector,Vector>> corridorSegments;
  IndexType triangle1Index, triangle2Index;
  for (std::size_t i=1; i<trianglesInCorridor.size(); ++i) {
    // find common edge between triangle i,i-1
    triangle1Index = trianglesInCorridor.at(i-1);
    triangle2Index = trianglesInCorridor.at(i);
    const auto sharedEdge = navmesh_.getSharedEdge(triangle1Index, triangle2Index);
    corridorSegments.emplace_back(sharedEdge.first, sharedEdge.second);
  }
  return corridorSegments;
}

// Free functions
template<typename NavmeshType>
std::vector<typename Pathfinder<NavmeshType>::IndexType> Pathfinder<NavmeshType>::rebuildPath(State state, const std::unordered_map<State, State> &previous) {
  std::vector<IndexType> result;
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

template<typename NavmeshType>
void Pathfinder<NavmeshType>::PathfindingResult::clear() {
  shortestPath.clear();
  aStarInfo.triangleCorridor.clear();
  aStarInfo.trianglesSearched.clear();
  aStarInfo.trianglesDiscovered.clear();
}