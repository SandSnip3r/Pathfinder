#ifndef PATHFINDER_NAVMESH_NAVMESH_ASTAR_STATE_H_
#define PATHFINDER_NAVMESH_NAVMESH_ASTAR_STATE_H_

#include <optional>
#include <ostream>

namespace pathfinder {

namespace navmesh {

template<typename IndexType>
class NavmeshAStarState {
public:
  NavmeshAStarState(IndexType triangleIndex);
  NavmeshAStarState(IndexType triangleIndex, IndexType entryEdgeIndex);
  IndexType getTriangleIndex() const;
  bool hasEntryEdgeIndex() const;
  IndexType getEntryEdgeIndex() const;
  bool isGoal() const;
  void setIsGoal(bool b);
  void setNewTriangleAndEntryEdge(const IndexType triangleIndex, const IndexType entryEdgeIndex);

  bool isSameTriangleAs(const NavmeshAStarState<IndexType> &otherState) const;

  template<typename IndexType2>
  friend bool operator<(const NavmeshAStarState<IndexType2> &s1, const NavmeshAStarState<IndexType2> &s2);
  template<typename IndexType2>
  friend bool operator==(const NavmeshAStarState<IndexType2> &s1, const NavmeshAStarState<IndexType2> &s2);
  template<typename IndexType2>
  friend std::ostream& operator<<(std::ostream& stream, const NavmeshAStarState<IndexType2> &state);

  friend struct std::hash<NavmeshAStarState<IndexType>>;

protected:
  IndexType triangleIndex_;
  std::optional<IndexType> entryEdgeIndex_;
  bool isGoal_{false};
};

template<typename IndexType>
bool operator!=(const NavmeshAStarState<IndexType> &s1, const NavmeshAStarState<IndexType> &s2) {
  return !(s1 == s2);
}

#include "navmesh_astar_state.inl"

} // namespace navmesh

} // namespace pathfinder

namespace std {

template <typename IndexType>
struct hash<pathfinder::navmesh::NavmeshAStarState<IndexType>> {
  std::size_t operator()(const pathfinder::navmesh::NavmeshAStarState<IndexType> &state) const {
    if (state.isGoal_) {
      return std::hash<bool>()(state.isGoal_);
    } else {
      return std::hash<std::optional<IndexType>>()(state.entryEdgeIndex_) ^
             (std::hash<IndexType>()(state.triangleIndex_) << 1);
    }
  }
};

} // namespace std

#endif // PATHFINDER_NAVMESH_NAVMESH_ASTAR_STATE_H_