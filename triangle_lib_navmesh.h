#ifndef PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_
#define PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_

#include "navmesh_astar_state.h"
#include "navmesh_types.h"
#include "vector.h"

#include "triangle/triangle_api.h"

#include <optional>
#include <utility>
#include <vector>

namespace pathfinder {

namespace navmesh {

class TriangleLibNavmesh : public NavmeshTypes<uint16_t> {
public:
  using State = NavmeshAStarState<IndexType>;
  using MarkerType = int;

  TriangleLibNavmesh(const triangle::triangleio &triangleData, const triangle::triangleio &triangleVoronoiData);
  std::size_t getVertexCount() const;
  Vector getVertex(const IndexType vertexIndex) const;
  MarkerType getVertexMarker(const IndexType vertexIndex) const;
  std::size_t getEdgeCount() const;
  EdgeType getEdge(const IndexType edgeIndex) const;
  EdgeVertexIndicesType getEdgeVertexIndices(const IndexType edgeIndex) const;
  MarkerType getEdgeMarker(const IndexType edgeIndex) const;
  std::size_t getTriangleCount() const;
  TriangleVertexIndicesType getTriangleVertexIndices(const IndexType triangleIndex) const;
  TriangleVerticesType getTriangleVertices(const IndexType triangleIndex) const;
  TriangleEdgeIndicesType getTriangleEdgeIndices(const IndexType triangleIndex) const;
  TriangleNeighborsAcrossEdgesIndicesType getTriangleNeighborsWithSharedEdges(const IndexType triangleIndex) const;
  EdgeType getSharedEdge(const IndexType triangle1Index, const IndexType triangle2Index) const;
  bool pointIsOnTriangle(const Vector &point, const IndexType triangleIndex) const;
  std::optional<IndexType> findTriangleForPoint(const Vector &point) const;
  std::vector<State> getSuccessors(const State &currentState, const State &goalState, const double agentRadius) const;
  // Debug help
  std::optional<IndexType> getVertexIndex(const Vector &vertex) const;

  static Vector to2dPoint(const Vector &point);
  static State createStartState(const Vector &startPoint, const IndexType startTriangle);
  static State createGoalState(const Vector &goalPoint, const IndexType goalTriangle);
protected:
  std::vector<Vector> vertices_;
  std::vector<EdgeVertexIndicesType> edgeVertexIndices_;
  std::vector<MarkerType> vertexMarkers_;
  std::vector<MarkerType> edgeMarkers_;
  std::vector<TriangleEdgeIndicesType> triangleEdgeIndices_;
  std::vector<TriangleVertexIndicesType> triangleVertexIndices_;
  // Sorted so that valid references are always first
  std::vector<TriangleNeighborIndicesType> triangleNeighborIndices_;
  std::vector<TriangleNeighborsAcrossEdgesIndicesType> triangleNeighborsAcrossEdgesIndices_;
};
  
} // namespace navmesh

} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_
