#include "a_star_navmesh_interface.h"

namespace pathfinder {

namespace navmesh {

bool operator==(const State &s1, const State &s2) {
  return (s1.isGoal() && s2.isGoal()) || (s1.getTriangleIndex() == s2.getTriangleIndex() && s1.getEntryEdgeIndex() == s2.getEntryEdgeIndex());
}

bool operator<(const State &s1, const State &s2) {
  if (!s1.isGoal() && !s2.isGoal()) {
    if (s1.getTriangleIndex() == s2.getTriangleIndex()) {
      return s1.getEntryEdgeIndex() < s2.getEntryEdgeIndex();
    } else {
      return s1.getTriangleIndex() < s2.getTriangleIndex();
    }
  } else {
    return (s1.isGoal() ? 1:0) < (s2.isGoal() ? 1:0);
  }
}

std::ostream& operator<<(std::ostream& stream, const State &state) {
  if (state.isGoal()) {
    stream << "[GOAL]";
  } else if (state.getEntryEdgeIndex() == -1) {
    stream << "[START]";
  } else {
    stream << '(' << state.getTriangleIndex() << ',' << state.getEntryEdgeIndex() << ')';
  }
  return stream;
}

} // namespace navmesh
  
} // namespace pathfinder