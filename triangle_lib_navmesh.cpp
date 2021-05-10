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
    if (vertexIndexA < 0 || vertexIndexA >= vertices_.size() ||
        vertexIndexB < 0 || vertexIndexB >= vertices_.size() ||
        vertexIndexC < 0 || vertexIndexC >= vertices_.size()) {
      throw std::runtime_error("Triangle references vertex that is out of bounds");
    }
    triangleVertexIndices_.emplace_back(vertexIndexA, vertexIndexB, vertexIndexC);
  }

  // Save neighbors of each triangle
  // TODO: Might be unneeded if the below `BEGIN TEST` is useful
  triangleNieghborIndices_.resize(triangleData.numberoftriangles);
  // Initialize all neighbors as -1
  std::for_each(triangleNieghborIndices_.begin(), triangleNieghborIndices_.end(), [](auto &edges) {
    std::get<0>(edges) = -1;
    std::get<1>(edges) = -1;
    std::get<2>(edges) = -1;
  });
  auto addNeighborForTriangle = [](auto &neighbors, const int neighborIndex) {
    if (std::get<0>(neighbors) == -1) {
      std::get<0>(neighbors) = neighborIndex;
    } else if (std::get<1>(neighbors) == -1) {
      std::get<1>(neighbors) = neighborIndex;
    } else if (std::get<2>(neighbors) == -1) {
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
      addNeighborForTriangle(triangleNieghborIndices_[triangleIndex], neighborIndex0);
    }
    if (neighborIndex1 > 0) {
      if (neighborIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Neighbor references triangle that is out of bounds");
      }
      addNeighborForTriangle(triangleNieghborIndices_[triangleIndex], neighborIndex1);
    }
    if (neighborIndex2 > 0) {
      if (neighborIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Neighbor references triangle that is out of bounds");
      }
      addNeighborForTriangle(triangleNieghborIndices_[triangleIndex], neighborIndex2);
    }
  }
  
  // Cache the edges of each triangle
  // TODO: Might be unneeded if the below `BEGIN TEST` is useful
  triangleEdgeIndices_.resize(triangleData.numberoftriangles);
  // Initialize all edges as -1
  std::for_each(triangleEdgeIndices_.begin(), triangleEdgeIndices_.end(), [](auto &edges) {
    std::get<0>(edges) = -1;
    std::get<1>(edges) = -1;
    std::get<2>(edges) = -1;
  });
  auto addEdgeForTriangle = [](auto &edges, const int edgeIndex) {
    if (std::get<0>(edges) == -1) {
      std::get<0>(edges) = edgeIndex;
    } else if (std::get<1>(edges) == -1) {
      std::get<1>(edges) = edgeIndex;
    } else if (std::get<2>(edges) == -1) {
      std::get<2>(edges) = edgeIndex;
    }
  };
  for (int voronoiEdgeIndex=0; voronoiEdgeIndex<triangleVoronoiData.numberofedges; ++voronoiEdgeIndex) {
    const int triangleIndex1 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2];
    const int triangleIndex2 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2+1];
    if (triangleIndex1 > 0) {
      if (triangleIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addEdgeForTriangle(triangleEdgeIndices_[triangleIndex1], voronoiEdgeIndex);
    }
    if (triangleIndex2 > 0) {
      if (triangleIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addEdgeForTriangle(triangleEdgeIndices_[triangleIndex2], voronoiEdgeIndex);
    }
  }

  // BEGIN TEST
  triangleNeighborsAcrossEdgesIndices_.resize(triangleData.numberoftriangles);
  std::for_each(triangleNeighborsAcrossEdgesIndices_.begin(), triangleNeighborsAcrossEdgesIndices_.end(), [](auto &neighorsAcrossEdge) {
    std::get<0>(neighorsAcrossEdge).neighborTriangleIndex = -1;
    std::get<0>(neighorsAcrossEdge).sharedEdgeIndex = -1;
    std::get<1>(neighorsAcrossEdge).neighborTriangleIndex = -1;
    std::get<1>(neighorsAcrossEdge).sharedEdgeIndex = -1;
    std::get<2>(neighorsAcrossEdge).neighborTriangleIndex = -1;
    std::get<2>(neighorsAcrossEdge).sharedEdgeIndex = -1;
  });
  auto addNeighborAcrossEdge = [](auto &neighorsAcrossEdge, const int triangleIndex, const int edgeIndex) {
    if (std::get<0>(neighorsAcrossEdge).neighborTriangleIndex == -1 && std::get<0>(neighorsAcrossEdge).sharedEdgeIndex == -1) {
      // First is unset
      std::get<0>(neighorsAcrossEdge).neighborTriangleIndex = triangleIndex;
      std::get<0>(neighorsAcrossEdge).sharedEdgeIndex = edgeIndex;
    } else if (std::get<1>(neighorsAcrossEdge).neighborTriangleIndex == -1 && std::get<1>(neighorsAcrossEdge).sharedEdgeIndex == -1) {
      // Second is unset
      std::get<1>(neighorsAcrossEdge).neighborTriangleIndex = triangleIndex;
      std::get<1>(neighorsAcrossEdge).sharedEdgeIndex = edgeIndex;
    } else if (std::get<2>(neighorsAcrossEdge).neighborTriangleIndex == -1 && std::get<2>(neighorsAcrossEdge).sharedEdgeIndex == -1) {
      // Third is unset
      std::get<2>(neighorsAcrossEdge).neighborTriangleIndex = triangleIndex;
      std::get<2>(neighorsAcrossEdge).sharedEdgeIndex = edgeIndex;
    }
  };
  for (int voronoiEdgeIndex=0; voronoiEdgeIndex<triangleVoronoiData.numberofedges; ++voronoiEdgeIndex) {
    const int triangleIndex1 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2];
    const int triangleIndex2 = triangleVoronoiData.edgelist[voronoiEdgeIndex*2+1];
    if (triangleIndex1 > 0) {
      if (triangleIndex1 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addNeighborAcrossEdge(triangleNeighborsAcrossEdgesIndices_[triangleIndex1], triangleIndex2, voronoiEdgeIndex);
    }
    if (triangleIndex2 > 0) {
      if (triangleIndex2 >= triangleData.numberoftriangles) {
        throw std::runtime_error("Voronoi edge references triangle that is out of bounds");
      }
      addNeighborAcrossEdge(triangleNeighborsAcrossEdgesIndices_[triangleIndex2], triangleIndex1, voronoiEdgeIndex);
    }
  }
}

size_t TriangleLibNavmesh::getVertexCount() const {
  return vertices_.size();
}

const Vector& TriangleLibNavmesh::getVertex(const int vertexIndex) const {
  if (vertexIndex < 0 || vertexIndex >= vertices_.size()) {
    throw std::runtime_error("Asking for vertex that is out of bounds");
  }
  return vertices_[vertexIndex];
}

unsigned int TriangleLibNavmesh::getVertexMarker(const int vertexIndex) const {
  if (vertexIndex < 0 || vertexIndex >= vertexMarkers_.size()) {
    throw std::runtime_error("Asking for vertex marker that is out of bounds");
  }
  return vertexMarkers_[vertexIndex];
}

size_t TriangleLibNavmesh::getEdgeCount() const {
  return edgeVertexIndices_.size();
}

EdgeType TriangleLibNavmesh::getEdge(const int edgeIndex) const {
  if (edgeIndex < 0 || edgeIndex >= edgeVertexIndices_.size()) {
    throw std::runtime_error("Asking for edge that is out of bounds");
  }
  const auto &edgeIndices = edgeVertexIndices_[edgeIndex];
  return {vertices_[edgeIndices.first], vertices_[edgeIndices.second]};
}

EdgeVertexIndicesType TriangleLibNavmesh::getEdgeVertexIndices(const int edgeIndex) const {
  if (edgeIndex < 0 || edgeIndex >= edgeVertexIndices_.size()) {
    throw std::runtime_error("Asking for edge that is out of bounds");
  }
  return edgeVertexIndices_[edgeIndex];
}

unsigned int TriangleLibNavmesh::getEdgeMarker(const int edgeIndex) const {
  if (edgeIndex < 0 || edgeIndex >= edgeMarkers_.size()) {
    throw std::runtime_error("Asking for edge marker that is out of bounds");
  }
  return edgeMarkers_[edgeIndex];
}

size_t TriangleLibNavmesh::getTriangleCount() const {
  return triangleVertexIndices_.size();
}

const TriangleVertexIndicesType& TriangleLibNavmesh::getTriangleVertexIndices(const int triangleIndex) const {
  if (triangleIndex < 0 || triangleIndex >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  return triangleVertexIndices_[triangleIndex];
}

const TriangleVerticesType& TriangleLibNavmesh::getTriangleVertices(const int triangleIndex) const {
  if (triangleIndex < 0 || triangleIndex >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  const auto &vertexIndices = triangleVertexIndices_[triangleIndex];
  return {vertices_[std::get<0>(vertexIndices)], vertices_[std::get<1>(vertexIndices)], vertices_[std::get<2>(vertexIndices)]};
}

const TriangleEdgeIndicesType& TriangleLibNavmesh::getTriangleEdgeIndices(const int triangleIndex) const {
  if (triangleIndex < 0 || triangleIndex >= triangleVertexIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  const auto &neighborsAcrossEdges = triangleNeighborsAcrossEdgesIndices_[triangleIndex];
  return {std::get<0>(neighborsAcrossEdges).sharedEdgeIndex, std::get<1>(neighborsAcrossEdges).sharedEdgeIndex, std::get<2>(neighborsAcrossEdges).sharedEdgeIndex};
}

const TriangleNeighborsAcrossEdgesIndicesType& TriangleLibNavmesh::getTriangleNeighborsWithSharedEdges(const int triangleIndex) const {
  if (triangleIndex < 0 || triangleIndex >= getTriangleCount()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  return triangleNeighborsAcrossEdgesIndices_[triangleIndex];
}

EdgeType TriangleLibNavmesh::getSharedEdge(const int triangle1Index, const int triangle2Index) const {
  if (triangle1Index < 0 || triangle1Index >= triangleNeighborsAcrossEdgesIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  if (triangle2Index < 0 || triangle2Index >= triangleNeighborsAcrossEdgesIndices_.size()) {
    throw std::runtime_error("Asking for triangle that is out of bounds");
  }
  const auto &neighborsAcrossEdges = triangleNeighborsAcrossEdgesIndices_[triangle1Index];
  int edgeIndex = -1;
  if (std::get<0>(neighborsAcrossEdges).neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<0>(neighborsAcrossEdges).sharedEdgeIndex;
  } else if (std::get<1>(neighborsAcrossEdges).neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<1>(neighborsAcrossEdges).sharedEdgeIndex;
  } else if (std::get<2>(neighborsAcrossEdges).neighborTriangleIndex == triangle2Index) {
    edgeIndex = std::get<2>(neighborsAcrossEdges).sharedEdgeIndex;
  }
  if (edgeIndex == -1) {
    throw std::runtime_error("There is no shared edge between triangle "+std::to_string(triangle1Index)+" and triangle "+std::to_string(triangle2Index));
  }
  const auto &edgeIndices = edgeVertexIndices_[edgeIndex];
  return {vertices_[edgeIndices.first], vertices_[edgeIndices.second]};
}

int TriangleLibNavmesh::getVertexIndex(const Vector &vertex) const {
  for (int i=0; i<vertices_.size(); ++i) {
    const auto &vertexInNavmesh = vertices_[i];
    if (vertex.x() == vertexInNavmesh.x() && vertex.y() == vertexInNavmesh.y()) {
      return i;
    }
  }
  return -1;
}

bool TriangleLibNavmesh::pointIsInTriangle(const Vector &point, const int triangleIndex) const {
  const auto &[vertex1, vertex2, vertex3] = getTriangleVertices(triangleIndex);
  // Triangles' vertices are listed in CCW order (might matter for checking if a point lies within a triangle)
  return math::isPointInTriangle(point, vertex1, vertex2, vertex3);
}

int TriangleLibNavmesh::findTriangleForPoint(const Vector &point) const {
  const auto triangleCount = getTriangleCount();
  for (int triangleIndex=0; triangleIndex<triangleCount; ++triangleIndex) {
    if (pointIsInTriangle(point, triangleIndex)) {
      return triangleIndex;
    }
  }
  return -1;
}

std::vector<State> TriangleLibNavmesh::getSuccessors(const State &currentState, const int goalTriangleIndex, const double agentRadius) const {
  if (currentState.isGoal()) {
    throw std::runtime_error("Trying to get successors of goal");
  }

  const auto triangleIndexForState = currentState.getTriangleIndex();
  if (triangleIndexForState < 0) {
    throw std::runtime_error("Current state triangle is invalid");
  }

  if (triangleIndexForState == goalTriangleIndex) {
    // This is the goal triangle, only successor is the goal point itself
    State goalState{currentState};
    goalState.setIsGoal(true);
    return {goalState};
  }

  if (triangleIndexForState >= getTriangleCount()) {
    throw std::runtime_error("Triangle is not in data");
  }

  std::vector<State> result;
  auto agentFitsThroughTriangle = [this, &agentRadius](const int triangleIndex, const int entryEdgeIndex, const int exitEdgeIndex) -> bool {
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

    if (entryEdgeIndex < 0) {
      // The agent already is inside this triangle, the only check we can do is make sure that the diameter is less than the width of the exit edge
      return math::distance(exitEdgeVertex1, exitEdgeVertex2) >= (agentRadius*2);
    }

    if (triangleIndex < 0 || triangleIndex >= getTriangleCount()) {
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
    const auto [entryEdgeVertex1Index, entryEdgeVertex2Index] = getEdgeVertexIndices(entryEdgeIndex);
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

    if (math::crossProduct(*vertexC, *vertexA, *vertexC, *vertexB) < 0.0) {
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
        if (edgeIndex >= 0) {
          const auto [edge1Vertex1Index, edge1Vertex2Index] = getEdgeVertexIndices(edgeIndex);
          if (edge1Vertex1Index == vertexBIndex && edge1Vertex2Index == vertexAIndex ||
              edge1Vertex1Index == vertexAIndex && edge1Vertex2Index == vertexBIndex) {
            // This is the opposite edge
            if (getEdgeMarker(edgeIndex) != 0) {
              oppositeEdgeIsConstrained = true;
            }
            break;
          }
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
  const auto &[neighborAcrossEdge1, neighborAcrossEdge2, neighborAcrossEdge3] = getTriangleNeighborsWithSharedEdges(triangleIndexForState);
  for (const auto &neighborAcrossEdge : {neighborAcrossEdge1, neighborAcrossEdge2, neighborAcrossEdge3}) {
    if (neighborAcrossEdge.neighborTriangleIndex >= 0) {
      // Neighbor exists
      if (neighborAcrossEdge.sharedEdgeIndex < 0) {
        throw std::runtime_error("Have neighbor triangle, but no shared edge between them");
      }
      if (neighborAcrossEdge.sharedEdgeIndex != currentState.getEntryEdgeIndex()) {
        // Not going back to previous
        if (getEdgeMarker(neighborAcrossEdge.sharedEdgeIndex) == 0) {
          // Non-constraint edge
          if (agentFitsThroughTriangle(triangleIndexForState, currentState.getEntryEdgeIndex(), neighborAcrossEdge.sharedEdgeIndex)) {
            State successor(neighborAcrossEdge.neighborTriangleIndex, neighborAcrossEdge.sharedEdgeIndex);
            result.push_back(successor);
          }
        }
      }
    }
  }
  return result;
}

} // namespace navmesh

} // namespace pathfinder
