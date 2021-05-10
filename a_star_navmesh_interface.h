#ifndef PATHFINDER_NAVMESH_A_STAR_NAVMESH_INTERFACE_H_
#define PATHFINDER_NAVMESH_A_STAR_NAVMESH_INTERFACE_H_

#include "navmesh_interface.h"

#include <iostream>
#include <vector>

namespace pathfinder {

namespace navmesh {

class State {
public:
  State() = default;
  State(int triangleIndex, int entryEdgeIndex=-1) : triangleIndex_(triangleIndex), entryEdgeIndex_(entryEdgeIndex) {}
  int getTriangleIndex() const {
    return triangleIndex_;
  }
  int getEntryEdgeIndex() const {
    return entryEdgeIndex_;
  }
  bool isGoal() const {
    return isGoal_;
  }
  void setIsGoal(bool b) {
    isGoal_ = b;
  }
private:
  int triangleIndex_{-1}, entryEdgeIndex_{-1};
  bool isGoal_{false};
};
bool operator==(const State &s1, const State &s2);
bool operator<(const State &s1, const State &s2);
std::ostream& operator<<(std::ostream& stream, const State &state);

/**
 * Interface of a Navmesh as used in an A* algorithm
 *
 * The A* navmesh provides an additional member function to get successor states for
 * a given triangle based on the edge that the triangle was entered through.
 *
 */
class AStarNavmeshInterface : public NavmeshInterface {
public:
  using NavmeshInterface::NavmeshInterface;
  /**
   * Returns a list of states that could be possible successors of the given state
   * 
   * @param currentState The current state to find successors of
   * @return A list of states that succeed the given state
   */
  virtual std::vector<State> getSuccessors(const State &currentState, const int goalTriangleIndex, const double agentRadius) const = 0;
};

} // namespace navmesh

} // namespace pathfinder

#endif // PATHFINDER_NAVMESH_A_STAR_NAVMESH_INTERFACE_H_