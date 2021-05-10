#ifndef PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_
#define PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_

#include "a_star_navmesh_interface.h"
#include "vector.h"

#include "triangle/triangle_api.h"

#include <utility>
#include <vector>

namespace pathfinder {

namespace navmesh {

class TriangleLibNavmesh : public AStarNavmeshInterface {
public:
  TriangleLibNavmesh(const triangle::triangleio &triangleData, const triangle::triangleio &triangleVoronoiData);
  virtual size_t getVertexCount() const;
  virtual const Vector& getVertex(const int vertexIndex) const;
  virtual unsigned int getVertexMarker(const int vertexIndex) const;
  virtual size_t getEdgeCount() const;
  virtual EdgeType getEdge(const int edgeIndex) const;
  virtual EdgeVertexIndicesType getEdgeVertexIndices(const int edgeIndex) const;
  virtual unsigned int getEdgeMarker(const int edgeIndex) const;
  virtual size_t getTriangleCount() const;
  virtual const TriangleVertexIndicesType& getTriangleVertexIndices(const int triangleIndex) const;
  virtual const TriangleVerticesType& getTriangleVertices(const int triangleIndex) const;
  virtual const TriangleEdgeIndicesType& getTriangleEdgeIndices(const int triangleIndex) const;
  virtual const TriangleNeighborsAcrossEdgesIndicesType& getTriangleNeighborsWithSharedEdges(const int triangleIndex) const;
  virtual EdgeType getSharedEdge(const int triangle1Index, const int triangle2Index) const;
  virtual int getVertexIndex(const Vector &vertex) const;
  virtual bool pointIsInTriangle(const Vector &point, const int triangleIndex) const;
  virtual int findTriangleForPoint(const Vector &point) const;
  virtual std::vector<State> getSuccessors(const State &currentState, const int goalTriangleIndex, const double agentRadius) const;
protected:
  std::vector<Vector> vertices_;
  std::vector<EdgeVertexIndicesType> edgeVertexIndices_;
  std::vector<unsigned int> vertexMarkers_;
  std::vector<unsigned int> edgeMarkers_;
  std::vector<TriangleEdgeIndicesType> triangleEdgeIndices_;
  std::vector<TriangleVertexIndicesType> triangleVertexIndices_;
  // Sorted so that valid references are always first
  std::vector<TriangleNeighborIndicesType> triangleNieghborIndices_;
  std::vector<TriangleNeighborsAcrossEdgesIndicesType> triangleNeighborsAcrossEdgesIndices_;
};
  
} // namespace navmesh

} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_TRIANGLE_LIB_NAVMESH_H_
