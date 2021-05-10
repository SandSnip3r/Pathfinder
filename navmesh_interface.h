#ifndef PATHFINDER_NAVMESH_NAVMESH_INTERFACE_H_
#define PATHFINDER_NAVMESH_NAVMESH_INTERFACE_H_

#include "vector.h"

#include <tuple>
#include <utility>

namespace pathfinder {

namespace navmesh {

using IndexType = int;

using EdgeType = std::pair<const Vector&, const Vector&>;
using EdgeVertexIndicesType = std::pair<IndexType, IndexType>;

using TriangleVerticesType = std::tuple<const Vector&, const Vector&, const Vector&>;
using TriangleVertexIndicesType = std::tuple<IndexType, IndexType, IndexType>;
using TriangleEdgeIndicesType = std::tuple<IndexType, IndexType, IndexType>;

// The indices of 3 neighbor triangles
using TriangleNeighborsIndicesType = std::tuple<IndexType, IndexType, IndexType>;

struct NeighborAcrossEdgeIndices {
  IndexType neighborTriangleIndex;
  IndexType sharedEdgeIndex;
};

// The indices of 3 neighboring triangles along with the index of the shared edge for each neighbor
using TriangleNeighborsAcrossEdgesIndicesType = std::tuple<NeighborAcrossEdgeIndices, NeighborAcrossEdgeIndices, NeighborAcrossEdgeIndices>;

/**
 * Interface of a Navmesh
 *
 * A navmesh contains a set of triangles that represent a 2d plane. These
 * triangles are made of vertices and edges. Vertex indices are [0, n)
 * with no gaps in between. Edge indices are [0, n) with no gaps in between.
 * Triangle indices are [0, n) with no gaps in between.
 *
 */
class NavmeshInterface {
public:
  /**
   * Returns the number of vertices in the navmesh.
   *
   * @return The number of vertices in the navmesh
   */
  virtual size_t getVertexCount() const = 0;

  /**
   * Returns a vertex of a triangle.
   *
   * @param vertexIndex The vertex index
   * @return A point representing the vertex of some triangle
   */
  virtual const Vector& getVertex(const int vertexIndex) const = 0;

  /**
   * Returns the marker for a vertex. This indicates what type of constraint this is.
   * 
   * @param vertexIndex The vertex index
   * @return The marker for the given vertex index
   */
  virtual unsigned int getVertexMarker(const int vertexIndex) const = 0;

  /**
   * Returns the number of edges in the navmesh.
   *
   * @return The number of edges in the navmesh
   */
  virtual size_t getEdgeCount() const = 0;

  /**
   * Returns an edge in the navmesh.
   *
   * @param edgeIndex The edge index
   * @return A pair of points defining the line segment that is the edge
   */
  virtual EdgeType getEdge(const int edgeIndex) const = 0;

  /**
   * Returns the indices of the vertices of an edge.
   *
   * @param edgeIndex The edge index
   * @return A pair of indices of the vertices that define the line segment that is the edge
   */
  virtual EdgeVertexIndicesType getEdgeVertexIndices(const int edgeIndex) const = 0;

  /**
   * Returns the marker for a edge. This indicates what type of constraint this is.
   * 
   * @param edgeIndex The edge index
   * @return The marker for the given edge index
   */
  virtual unsigned int getEdgeMarker(const int edgeIndex) const = 0;

  /**
   * Returns the number of triangles in the navmesh.
   *
   * @return The number of triangles in the navmesh
   */
  virtual size_t getTriangleCount() const = 0;

  /**
   * Returns the indices of the vertices of a triangle.
   *
   * @param triangleIndex The triangle index
   * @return A tuple of indices of the vertices for the given triangle
   */
  virtual const TriangleVertexIndicesType& getTriangleVertexIndices(const int triangleIndex) const = 0;

  /**
   * Returns the vertices of a triangle.
   *
   * @param triangleIndex The triangle index
   * @return A tuple of vertices for the given triangle
   */
  virtual const TriangleVerticesType& getTriangleVertices(const int triangleIndex) const = 0;

  /**
   * Returns the indices of the edges of a triangle.
   *
   * @param triangleIndex The triangle index
   * @return A tuple of indices of the edges for the given triangle
   */
  virtual const TriangleEdgeIndicesType& getTriangleEdgeIndices(const int triangleIndex) const = 0;

  /**
   * Returns indices of the neighboring triangles for the given triangle along with the indices of the edge shared between the given triangle and the neighbor.
   * 
   * @param triangleIndex The triangle index
   * @return The 3 neighboring triangle indices (-1 if no neighbor) and index of the shared edge between the given triangle and the neighbor (undefined if no neighbor, (TODO: Maybe a bad idea?)).
   */
  virtual const TriangleNeighborsAcrossEdgesIndicesType& getTriangleNeighborsWithSharedEdges(const int triangleIndex) const = 0;

  /**
   * Returns the shared edge between the two given triangles.
   * 
   * Throws if invalid triangle given.
   * Throws if no edge exists between the two triangles.
   * 
   * @param triangle1Index The triangle index
   * @param triangle2Index The triangle index
   * @return The edge between the the two given triangles
   */
  virtual EdgeType getSharedEdge(const int triangle1Index, const int triangle2Index) const = 0;

  /**
   * Returns the index of the vertex in the navmesh that matches the given vertex
   * 
   * Note: Only exists for debugging. TODO: Remove
   * 
   * @param vertex The vertex to search for
   * @return The index of the vertex if it exists, -1 otherwise
   */
  virtual int getVertexIndex(const Vector &vertex) const = 0;

  /**
   * Checks if a given point is inside of a triangle.
   *
   * @param point The point
   * @param triangleIndex The triangle index
   * @return `true` if the point is in the triangle, `false` otherwise.
   */
  virtual bool pointIsInTriangle(const Vector &point, const int triangleIndex) const = 0;

  /**
   * Returns the index of the triangle that the given point is in
   * 
   * @param point The point to find a triangle for
   * @return The index of the triangle if it exists, -1 otherwise
   */
  virtual int findTriangleForPoint(const Vector &point) const = 0;
};

} // namespace navmesh

} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_NAVMESH_INTERFACE_H_