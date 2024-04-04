#ifndef PATHFINDER_NAVMESH_NAVMESH_ASTAR_STATE_H_
#define PATHFINDER_NAVMESH_NAVMESH_ASTAR_STATE_H_

#include <absl/strings/str_format.h>

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

  std::string toString() const;

  template<typename IndexType2>
  friend bool operator<(const NavmeshAStarState<IndexType2> &s1, const NavmeshAStarState<IndexType2> &s2);
  template<typename IndexType2>
  friend bool operator==(const NavmeshAStarState<IndexType2> &s1, const NavmeshAStarState<IndexType2> &s2);

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

template<typename IndexType>
NavmeshAStarState<IndexType>::NavmeshAStarState(IndexType triangleIndex) : triangleIndex_(triangleIndex) {}

template<typename IndexType>
NavmeshAStarState<IndexType>::NavmeshAStarState(IndexType triangleIndex, IndexType entryEdgeIndex) : triangleIndex_(triangleIndex), entryEdgeIndex_(entryEdgeIndex) {}

template<typename IndexType>
IndexType NavmeshAStarState<IndexType>::getTriangleIndex() const {
  return triangleIndex_;
}

template<typename IndexType>
bool NavmeshAStarState<IndexType>::hasEntryEdgeIndex() const {
  return entryEdgeIndex_.has_value();
}

template<typename IndexType>
IndexType NavmeshAStarState<IndexType>::getEntryEdgeIndex() const {
  if (!entryEdgeIndex_.has_value()) {
    throw std::runtime_error("Trying to get entry edge index, but there is none");
  }
  return *entryEdgeIndex_;
}

template<typename IndexType>
bool NavmeshAStarState<IndexType>::isGoal() const {
  return isGoal_;
}

template<typename IndexType>
void NavmeshAStarState<IndexType>::setIsGoal(bool b) {
  isGoal_ = b;
}

template<typename IndexType>
void NavmeshAStarState<IndexType>::setNewTriangleAndEntryEdge(const IndexType triangleIndex, const IndexType entryEdgeIndex) {
  triangleIndex_ = triangleIndex;
  entryEdgeIndex_ = entryEdgeIndex;
}

template<typename IndexType>
bool NavmeshAStarState<IndexType>::isSameTriangleAs(const NavmeshAStarState<IndexType> &otherState) const {
  return (triangleIndex_ == otherState.triangleIndex_);
}

template<typename IndexType>
std::string NavmeshAStarState<IndexType>::toString() const {
  return absl::StrFormat("(%s%d,%d)", (isGoal_ ? "GOAL," : (!entryEdgeIndex_ ? "START," : "")), triangleIndex_, entryEdgeIndex_.value_or(-1));
}

template<typename IndexType>
bool operator==(const NavmeshAStarState<IndexType> &s1, const NavmeshAStarState<IndexType> &s2) {
  if (s1.isGoal_ && s2.isGoal_) {
    // Both are the goal
    return true;
  } else if (s1.isGoal_ != s2.isGoal_) {
    // One is the goal and one is not
    return false;
  } else {
    // Both are not goal
    return (s1.triangleIndex_ == s2.triangleIndex_ && s1.entryEdgeIndex_ == s2.entryEdgeIndex_);
  }
}

template<typename IndexType>
bool operator<(const NavmeshAStarState<IndexType> &s1, const NavmeshAStarState<IndexType> &s2) {
  if (!s1.isGoal_ && !s2.isGoal_) {
    if (s1.triangleIndex_ == s2.triangleIndex_) {
      // Compare entry edges
      return s1.entryEdgeIndex_ < s2.entryEdgeIndex_;
    } else {
      return s1.triangleIndex_ < s2.triangleIndex_;
    }
  } else {
    return (s1.isGoal_ ? 1:0) < (s2.isGoal_ ? 1:0);
  }
}

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