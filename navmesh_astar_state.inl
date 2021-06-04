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

template<typename IndexType>
std::ostream& operator<<(std::ostream& stream, const NavmeshAStarState<IndexType> &state) {
  if (state.isGoal_) {
    stream << "[GOAL]";
  } else if (!state.hasEntryEdgeIndex()) {
    stream << "[START]";
  } else {
    stream << '(' << state.getTriangleIndex() << ',' << state.getEntryEdgeIndex() << ')';
  }
  return stream;
}