#include "math_helpers.h"
#include "triangle_lib_navmesh.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace pathfinder {

namespace navmesh {

TriangleLibNavmesh::TriangleLibNavmesh(const triangle::triangleio &triangleData, const triangle::triangleio &triangleVoronoiData) {
  // Check triangle data for internal consistency
  if (triangleData.trianglelist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: trianglelist is null");
  }
  if (triangleData.pointlist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: pointlist is null");
  }
  if (triangleData.edgelist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: edgelist is null");
  }
  if (triangleData.pointmarkerlist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: pointmarkerlist is null");
  }
  if (triangleData.edgemarkerlist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: edgemarkerlist is null");
  }
  if (triangleVoronoiData.edgelist == nullptr) {
    throw std::runtime_error("TriangleLibNavmesh: voronoi edgelist is null");
  }

  using CommonType = std::common_type_t<int, IndexType>;
  if (static_cast<CommonType>(triangleData.numberofpoints) > static_cast<CommonType>(std::numeric_limits<IndexType>::max())) {
    // We will later try to pack this into a special integer, this requires every index fits in 16 bits
    throw std::runtime_error("TriangleLibNavmesh: Too many points");
  }
  if (static_cast<CommonType>(triangleData.numberofsegments) > static_cast<CommonType>(std::numeric_limits<IndexType>::max())) {
    // We will later try to pack this into a special integer, this requires every index fits in 16 bits
    throw std::runtime_error("TriangleLibNavmesh: Too many segments");
  }
  if (static_cast<CommonType>(triangleData.numberoftriangles) > static_cast<CommonType>(std::numeric_limits<IndexType>::max())) {
    // We will later try to pack this into a special integer, this requires every index fits in 16 bits
    throw std::runtime_error("TriangleLibNavmesh: Too many triangles");
  }
  if (static_cast<CommonType>(triangleData.numberofedges) > static_cast<CommonType>(std::numeric_limits<IndexType>::max())) {
    // We will later try to pack this into a special integer, this requires every index fits in 16 bits
    throw std::runtime_error("TriangleLibNavmesh: Too many edges");
  }

  // # Extract data into internal format
  
  // Save all vertices and their markers
  vertices_.reserve(triangleData.numberofpoints);
  vertexMarkers_.reserve(triangleData.numberofpoints);
  for (int vertexIndex=0; vertexIndex<triangleData.numberofpoints; ++vertexIndex) {
    vertices_.emplace_back(triangleData.pointlist[vertexIndex*2], triangleData.pointlist[vertexIndex*2+1]);
    vertexMarkers_.emplace_back(triangleData.pointmarkerlist[vertexIndex]);
  }
  
  // Save all edges and their markers
  edgeVertexIndices_.reserve(triangleData.numberofsegments);
  edgeMarkers_.reserve(triangleData.numberofsegments);
  for (int edgeIndex=0; edgeIndex<triangleData.numberofedges; ++edgeIndex) {
    const auto vertexAIndex = triangleData.edgelist[edgeIndex*2];
    const auto vertexBIndex = triangleData.edgelist[edgeIndex*2+1];
    if (vertexAIndex < 0 || vertexAIndex >= triangleData.numberofpoints || vertexBIndex < 0 || vertexBIndex >= triangleData.numberofpoints) {
    // Validate that all data referenced is valid
      throw std::runtime_error("Edge references vertex that is out of bounds");
    }
    edgeVertexIndices_.emplace_back(vertexAIndex, vertexBIndex);
    edgeMarkers_.emplace_back(triangleData.edgemarkerlist[edgeIndex]);
  }

  // Save vertices of each triangle
  triangleVertexIndices_.reserve(triangleData.numberoftriangles);
  for (int triangleIndex=0; triangleIndex<triangleData.numberoftriangles; ++triangleIndex) {
    const auto vertexIndexA = triangleData.trianglelist[triangleIndex*3];
    const auto vertexIndexB = triangleData.trianglelist[triangleIndex*3+1];
    const auto vertexIndexC = triangleData.trianglelist[triangleIndex*3+2];
    if (vertexIndexA < 0 || static_cast<std::size_t>(vertexIndexA) >= vertices_.size() ||
        vertexIndexB < 0 || static_cast<std::size_t>(vertexIndexB) >= vertices_.size() ||
        vertexIndexC < 0 || static_cast<std::size_t>(vertexIndexC) >= vertices_.size()) {
      throw std::runtime_error("Triangle references vertex that is out of bounds");
    }
    triangleVertexIndices_.emplace_back(vertexIndexA, vertexIndexB, vertexIndexC);
  }

  // Save neighbors of each triangle
  // TODO: Might be unneeded if the below `BEGIN TEST` is useful
  triangleNeighborIndices_.resize(triangleData.numberoftriangles);
  auto addNeighborForTriangle = [](auto &neighbors, const int neighborIndex) {
    if (!std::get<0>(neighbors).has_value()) {
      std::get<0>(neighbors) = neighborIndex;
    } else if (!std::get<1>(neighbors).has_value()) {
      std::get<1>(neighbors) = neighborIndex;
    } else if (!std::get<2>(neighbors).has_value()) {
      std::get<2>(neighbors) = neighborIndex;
    }
  };
  for (int triangleIndex=0; triangleIndex<triangleData.numberoftriangles; ++triangleIndex) {
    const auto neighborIndex0 = triangleData.neighborlist[triangleIndex*3];
    const auto neighborIndex1 = triangleData.neighborlist[triangleIndex*3+1];
    const auto neighborIndex2 = triangleData.neighborlist[triangleIndex*3+1];
    if (neighborIndex0 > 0) {
      if (neighborIndex0 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Neighbor references triangle that is out of bounds");
      }
      addNeighborForTriangle(triangleNeighborIndices_[triangleIndex], neighborIndex0);
    }
    if (neighborIndex1 > 0) {
      if (neighborIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Neighbor references triangle that is out of bounds");
      }
      addNeighborForTriangle(triangleNeighborIndices_[triangleIndex], neighborIndex1);
    }
    if (neighborIndex2 > 0) {
      if (neighborIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Neighbor references triangle that is out of bounds");
      }
      addNeighborForTriangle(triangleNeighborIndices_[triangleIndex], neighborIndex2);
    }
  }
  
  // Cache the edges of each triangle
  // TODO: Might be unneeded if the below `BEGIN TEST` is useful
  triangleEdgeIndices_.resize(triangleData.numberoftriangles);
  std::vector<int> edgeCountForTriangle(triangleData.numberoftriangles, 0);
  auto addEdgeForTriangle = [](auto &edges, int &edgeForTriangle, const int edgeIndex) {
    if (edgeForTriangle == 0) {
      std::get<0>(edges) = edgeIndex;
    } else if (edgeForTriangle == 1) {
      std::get<1>(edges) = edgeIndex;
    } else if (edgeForTriangle == 2) {
      std::get<2>(edges) = edgeIndex;
    } else {
      throw std::runtime_error("Triangle has too many edges");
    }
    ++edgeForTriangle;
  };
  for (int voronoiEdgeIndex=0; voronoiEdgeIndex<triangleVoronoiData.numberofedges; ++voronoiEdgeIndex) {
    const int triangleIndex1 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2];
    const int triangleIndex2 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2+1];
    if (triangleIndex1 >= 0) {
      if (triangleIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addEdgeForTriangle(triangleEdgeIndices_[triangleIndex1], edgeCountForTriangle[triangleIndex1], voronoiEdgeIndex);
    }
    if (triangleIndex2 >= 0) {
      if (triangleIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addEdgeForTriangle(triangleEdgeIndices_[triangleIndex2], edgeCountForTriangle[triangleIndex2], voronoiEdgeIndex);
    }
  }

  // BEGIN TEST
  triangleNeighborsAcrossEdgesIndices_.resize(triangleData.numberoftriangles);
  auto setNeighborAcrossEdge = [](auto &neighborsAcrossEdge, const IndexType triangleIndex, const IndexType edgeIndex) {
    if (!std::get<0>(neighborsAcrossEdge).has_value()) {
      // First is unset
      std::get<0>(neighborsAcrossEdge) = {triangleIndex, edgeIndex};
    } else if (!std::get<1>(neighborsAcrossEdge).has_value()) {
      // Second is unset
      std::get<1>(neighborsAcrossEdge) = {triangleIndex, edgeIndex};
    } else if (!std::get<2>(neighborsAcrossEdge).has_value()) {
      // Third is unset
      std::get<2>(neighborsAcrossEdge) = {triangleIndex, edgeIndex};
    }
  };
  for (int voronoiEdgeIndex=0; voronoiEdgeIndex<triangleVoronoiData.numberofedges; ++voronoiEdgeIndex) {
    const int triangleIndex1 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2];
    const int triangleIndex2 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2+1];
    if (triangleIndex1 >= 0 && triangleIndex2 >= 0) {
      if (triangleIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      if (triangleIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      setNeighborAcrossEdge(triangleNeighborsAcrossEdgesIndices_[triangleIndex1], static_cast<IndexType>(triangleIndex2), static_cast<IndexType>(voronoiEdgeIndex));
      setNeighborAcrossEdge(triangleNeighborsAcrossEdgesIndices_[triangleIndex2], static_cast<IndexType>(triangleIndex1), static_cast<IndexType>(voronoiEdgeIndex));
    }
  }
}

std::size_t TriangleLibNavmesh::getVertexCount() const {
  return vertices_.size();
}

Vector TriangleLibNavmesh::getVertex(const IndexType vertexIndex) const {
  if (static_cast<std::size_t>(vertexIndex) >= vertices_.size()) {
    throw std::runtime_error("Asking for vertex that is out of bounds");
  }
  return vertices_[vertexIndex];
}

TriangleLibNavmesh::MarkerType TriangleLibNavmesh::getVertexMarker(const IndexType vertexIndex) const {
  if (static_cast<std::size_t>(vertexIndex) >= vertexMarkers_.size()) {
    throw std::runtime_error("Asking for vertex marker that is out of bounds");
  }
  return vertexMarkers_[vertexIndex];
}

std::size_t TriangleLibNavmesh::getEdgeCount() const {
  return edgeVertexIndices_.size();
}

TriangleLibNavmesh::EdgeType TriangleLibNavmesh::getEdge(const IndexType edgeIndex) const {
  if (static_cast<std::size_t>(edgeIndex) >= edgeVertexIndices_.size()) {
    throw std::runtime_error("Asking for edge that is out of bounds");
  }
  const auto &edgeIndices = edgeVertexIndices_[edgeIndex];
  return {vertices_[edgeIndices.first], vertices_[edgeIndices.second]};
}

TriangleLibNavmesh::EdgeVertexIndicesType TriangleLibNavmesh::getEdgeVertexIndices(const IndexType edgeIndex) const {
  if (static_cast<std::size_t>(edgeIndex) >= edgeVertexIndices_.size()) {
    throw std::runtime_error("Asking for edge that is out of bounds");
  }
  return edgeVertexIndices_[edgeIndex];
}

TriangleLibNavmesh::MarkerType TriangleLibNavmesh::getEdgeMarker(const IndexType edgeIndex) const {
  if (static_cast<std::size_t>(edgeIndex) >= edgeMarkers_.size()) {
    throw std::runtime_error("Asking for edge marker that is out of bounds");
  }
  return edgeMarkers_[edgeIndex];
}

std::size_t TriangleLibNavmesh::getTriangleCount() const {
  return triangleVertexIndices_.size();
}

TriangleLibNavmesh::TriangleVertexIndicesType TriangleLibNavmesh::getTriangleVertexIndices(const IndexType triangleIndex) const {
  if (static_cast<std::size_t>(triangleIndex) >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  return triangleVertexIndices_[triangleIndex];
}

TriangleLibNavmesh::TriangleVerticesType TriangleLibNavmesh::getTriangleVertices(const IndexType triangleIndex) const {
  if (static_cast<std::size_t>(triangleIndex) >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  const auto &vertexIndices = triangleVertexIndices_[triangleIndex];
  return {vertices_[std::get<0>(vertexIndices)], vertices_[std::get<1>(vertexIndices)], vertices_[std::get<2>(vertexIndices)]};
}

TriangleLibNavmesh::TriangleEdgeIndicesType TriangleLibNavmesh::getTriangleEdgeIndices(const IndexType triangleIndex) const {
  if (static_cast<std::size_t>(triangleIndex) >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  return triangleEdgeIndices_[triangleIndex];
}

TriangleLibNavmesh::TriangleNeighborsAcrossEdgesIndicesType TriangleLibNavmesh::getTriangleNeighborsWithSharedEdges(const IndexType triangleIndex) const {
  if (static_cast<std::size_t>(triangleIndex) >= getTriangleCount()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  return triangleNeighborsAcrossEdgesIndices_[triangleIndex];
}

TriangleLibNavmesh::EdgeType TriangleLibNavmesh::getSharedEdge(const IndexType triangle1Index, const IndexType triangle2Index) const {
  if (static_cast<std::size_t>(triangle1Index) >= triangleNeighborsAcrossEdgesIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  if (static_cast<std::size_t>(triangle2Index) >= triangleNeighborsAcrossEdgesIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  const auto &neighborsAcrossEdges = triangleNeighborsAcrossEdgesIndices_[triangle1Index];
  std::optional<IndexType> edgeIndex;
  if (std::get<0>(neighborsAcrossEdges).has_value() && std::get<0>(neighborsAcrossEdges)->neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<0>(neighborsAcrossEdges)->sharedEdgeIndex;
  } else if (std::get<1>(neighborsAcrossEdges).has_value() && std::get<1>(neighborsAcrossEdges)->neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<1>(neighborsAcrossEdges)->sharedEdgeIndex;
  } else if (std::get<2>(neighborsAcrossEdges).has_value() && std::get<2>(neighborsAcrossEdges)->neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<2>(neighborsAcrossEdges)->sharedEdgeIndex;
  }
  if (!edgeIndex.has_value()) {
    throw std::runtime_error("There is no shared edge between triangle "+std::to_string(triangle1Index)+" and triangle "+std::to_string(triangle2Index));
  }
  const auto &edgeIndices = edgeVertexIndices_[*edgeIndex];
  return {vertices_[edgeIndices.first], vertices_[edgeIndices.second]};
}

std::optional<TriangleLibNavmesh::IndexType> TriangleLibNavmesh::getVertexIndex(const Vector &vertex) const {
  for (std::size_t i=0; i<vertices_.size(); ++i) {
    const auto &vertexInNavmesh = vertices_[i];
    if (math::equal(vertex, vertexInNavmesh)) {
      return static_cast<IndexType>(i);
    }
  }
  return {};
}

bool TriangleLibNavmesh::pointIsOnTriangle(const Vector &point, const IndexType triangleIndex) const {
  const auto &[vertex1, vertex2, vertex3] = getTriangleVertices(triangleIndex);
  // Triangles' vertices are listed in CCW order (might matter for checking if a point lies within a triangle)
  return math::isPointOnTriangle(point, vertex1, vertex2, vertex3);
}

std::optional<TriangleLibNavmesh::IndexType> TriangleLibNavmesh::findTriangleForPoint(const Vector &point) const {
  const auto triangleCount = getTriangleCount();
  for (std::size_t triangleIndex=0; triangleIndex<triangleCount; ++triangleIndex) {
    if (pointIsOnTriangle(point, static_cast<IndexType>(triangleIndex))) {
      return static_cast<IndexType>(triangleIndex);
    }
  }

  // No triangle found
  return {};
}

std::vector<TriangleLibNavmesh::State> TriangleLibNavmesh::getSuccessors(const State &currentState, const State &goalState, const double agentRadius) const {
  if (currentState.isGoal()) {
    throw std::runtime_error("Trying to get successors of goal");
  }

  if (currentState.isSameTriangleAs(goalState)) {
    // This is the goal, only successor is the goal point itself
    State newGoalState{currentState};
    newGoalState.setIsGoal(true);
    return {newGoalState};
  }

  const auto triangleIndexForCurrentState = currentState.getTriangleIndex();
  if (triangleIndexForCurrentState >= getTriangleCount()) {
    throw std::runtime_error("Triangle is not in data");
  }

  std::vector<State> result;
  auto agentFitsThroughTriangle = [this, &agentRadius](const IndexType triangleIndex, const std::optional<IndexType> entryEdgeIndex, const int exitEdgeIndex) -> bool {
    // TODO: Need to improve
    //  Some of these edges might not be constrained
    //  See C:\Users\Victor\Documents\ShareX\Screenshots\2021-02\drawPolygon_lYhSzArwEG.png
    //  Every point is on a constraint, but not necessarily every edge is
    if (agentRadius == 0.0) {
      return true;
    }

    // Get the exiting edge for this triangle
    const auto [exitEdgeVertex1Index, exitEdgeVertex2Index] = getEdgeVertexIndices(exitEdgeIndex);
    const auto& exitEdgeVertex1 = getVertex(exitEdgeVertex1Index);
    const auto& exitEdgeVertex2 = getVertex(exitEdgeVertex2Index);

    if (!entryEdgeIndex) {
      // The agent already is inside this triangle, the only check we can do is make sure that the diameter is less than the width of the exit edge
      return math::distance(exitEdgeVertex1, exitEdgeVertex2) >= (agentRadius*2);
    }

    if (triangleIndex >= getTriangleCount()) {
      throw std::runtime_error("Referencing invalid triangle");
    }

    {
      // Check if all vertices of this triangle are part of constraints
      const auto [vertex1Index, vertex2Index, vertex3Index] = getTriangleVertexIndices(triangleIndex);
      if (getVertexMarker(vertex1Index) == 0 ||
          getVertexMarker(vertex2Index) == 0 ||
          getVertexMarker(vertex3Index) == 0) {
        // One of the vertices is not part of a constraint, not yet handling this case
        // TODO
        return true;
      }
    }

    // Get the entry edge for this triangle
    const auto [entryEdgeVertex1Index, entryEdgeVertex2Index] = getEdgeVertexIndices(*entryEdgeIndex);
    const auto &entryEdgeVertex1 = getVertex(entryEdgeVertex1Index);
    const auto &entryEdgeVertex2 = getVertex(entryEdgeVertex2Index);

    int vertexAIndex, vertexBIndex;
    const Vector *vertexA{nullptr}, *vertexB{nullptr}, *vertexC{nullptr};
    if (entryEdgeVertex1Index == exitEdgeVertex1Index) {
      vertexC = &entryEdgeVertex1;

      vertexB = &entryEdgeVertex2;
      vertexBIndex = entryEdgeVertex2Index;

      vertexA = &exitEdgeVertex2;
      vertexAIndex = exitEdgeVertex2Index;
    } else if (entryEdgeVertex1Index == exitEdgeVertex2Index) {
      vertexC = &entryEdgeVertex1;

      vertexB = &entryEdgeVertex2;
      vertexBIndex = entryEdgeVertex2Index;

      vertexA = &exitEdgeVertex1;
      vertexAIndex = exitEdgeVertex1Index;
    } else if (entryEdgeVertex2Index == exitEdgeVertex1Index) {
      vertexC = &entryEdgeVertex2;

      vertexB = &entryEdgeVertex1;
      vertexBIndex = entryEdgeVertex1Index;

      vertexA = &exitEdgeVertex2;
      vertexAIndex = exitEdgeVertex2Index;
    } else if (entryEdgeVertex2Index == exitEdgeVertex2Index) {
      vertexC = &entryEdgeVertex2;

      vertexB = &entryEdgeVertex1;
      vertexBIndex = entryEdgeVertex1Index;

      vertexA = &exitEdgeVertex1;
      vertexAIndex = exitEdgeVertex1Index;
    } else {
      throw std::runtime_error("No shared vertex between two edges in triangle");
    }

    if (vertexA == nullptr || vertexB == nullptr || vertexC == nullptr) {
      throw std::runtime_error("Error while assigining vertices");
    }

    // `vertexC` is the common vertex between the entry and exit edge of this triangle

    if (math::crossProductForSign(*vertexC, *vertexA, *vertexC, *vertexB) < 0.0) {
      // We want A to be to the right of B
      std::swap(vertexA, vertexB);
      std::swap(vertexAIndex, vertexBIndex);
    }

    // `entryEdge` and `exitEdge` are CA and CB (or swapped)
    const double cabAngle = math::angleBetweenVectors(*vertexA, *vertexC, *vertexA, *vertexB);
    const double cbaAngle = math::angleBetweenVectors(*vertexB, *vertexA, *vertexB, *vertexC);
    if (cabAngle >= math::kPi/2.0) {
      // Angle CAB is right or obtuse, the closest constraint is vertex A
      return (math::distance(*vertexC, *vertexA) >= (agentRadius*2));
    } else if (cbaAngle >= math::kPi/2.0) {
      // Angle CBA is right or obtuse, the closest constraint is vertex B
      return (math::distance(*vertexC, *vertexB) >= (agentRadius*2));
    } else {
      // Both CAB and CBA are acute
      // Need to worry about colliding with edge opposite to common vertex
      bool oppositeEdgeIsConstrained{false};
      const auto [edge1Index, edge2Index, edge3Index] = getTriangleEdgeIndices(triangleIndex);
      for (const auto &edgeIndex : {edge1Index, edge2Index, edge3Index}) {
        const auto [edge1Vertex1Index, edge1Vertex2Index] = getEdgeVertexIndices(edgeIndex);
        if ((edge1Vertex1Index == vertexBIndex && edge1Vertex2Index == vertexAIndex) ||
            (edge1Vertex1Index == vertexAIndex && edge1Vertex2Index == vertexBIndex)) {
          // This is the opposite edge
          if (getEdgeMarker(edgeIndex) != 0) {
            oppositeEdgeIsConstrained = true;
          }
          break;
        }
      }
      if (oppositeEdgeIsConstrained) {
        // Check if the agent fits betweeen the shared vertex and the edge
        return (math::distanceBetweenEdgeAndPoint(*vertexB, *vertexA, *vertexC) >= (agentRadius*2));
      } else {
        // Opposite edge is not constrained, not yet handling this case
        // TODO
        // However, since the vertices are still constraints, we do know that these are upperbounds for the size of agent that can fit
        return (math::distance(*vertexC, *vertexA) >= (agentRadius*2)) && (math::distance(*vertexC, *vertexB) >= (agentRadius*2));
      }
    }
  };
  
  // For each neighboring triangle
  const auto &[neighborAcrossEdge1, neighborAcrossEdge2, neighborAcrossEdge3] = getTriangleNeighborsWithSharedEdges(triangleIndexForCurrentState);
  for (const auto &neighborAcrossEdge : {neighborAcrossEdge1, neighborAcrossEdge2, neighborAcrossEdge3}) {
    if (neighborAcrossEdge) {
      // Neighbor exists
      if (!currentState.hasEntryEdgeIndex() || neighborAcrossEdge->sharedEdgeIndex != currentState.getEntryEdgeIndex()) {
        // Not going back to previous
        if (getEdgeMarker(neighborAcrossEdge->sharedEdgeIndex) == 0) {
          // Non-constraint edge
          std::optional<IndexType> entryEdgeIndex;
          if (currentState.hasEntryEdgeIndex()) {
            entryEdgeIndex = currentState.getEntryEdgeIndex();
          }
          if (agentFitsThroughTriangle(triangleIndexForCurrentState, entryEdgeIndex, neighborAcrossEdge->sharedEdgeIndex)) {
            State successor(neighborAcrossEdge->neighborTriangleIndex, neighborAcrossEdge->sharedEdgeIndex);
            result.push_back(successor);
          }
        }
      }
    }
  }
  return result;
}

Vector TriangleLibNavmesh::to2dPoint(const Vector &point) {
  return point;
}

TriangleLibNavmesh::State TriangleLibNavmesh::createStartState(const Vector &startPoint, const IndexType startTriangle) {
  (void)startPoint;
  State result{startTriangle};
  return result;
}

TriangleLibNavmesh::State TriangleLibNavmesh::createGoalState(const Vector &goalPoint, const IndexType goalTriangle) {
  (void)goalPoint;
  State result{goalTriangle};
  result.setIsGoal(true);
  return result;
}

} // namespace navmesh

} // namespace pathfinder
