#ifndef BEHAVIOR_BUILDER_H_
#define BEHAVIOR_BUILDER_H_

#include "triangle/triangle.h"

namespace pathfinder {

class BehaviorBuilder {
public:
  BehaviorBuilder();

  triangle::behavior getBehavior() const;

  void setConformingDelaunay(bool val);
  void setEnforceMinimumAngle(bool val, float angle=20.0);
  void setMinimumAngle(float angle);
  void setEnforceMinimumArea(bool val, float area=2000.0);
  void setMinimumArea(float area);

private:
  triangle::behavior behavior_;
};

} // namespace pathfinder

#endif // BEHAVIOR_BUILDER_H_