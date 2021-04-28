#include "behaviorBuilder.h"

#include <cmath>

namespace pathfinder {

BehaviorBuilder::BehaviorBuilder() {
  // Set a bunch of default behavior

  // do want a PSLG triangulation
  behavior_.poly = 1;

  // not refining
  behavior_.refine = 0;
  
  // no quality mesh generation
  //  i think this adds extra points
  behavior_.quality = 0;

  // no triangle area constraints
  behavior_.vararea = 0;
  behavior_.fixedarea = 0;
  behavior_.usertest = 0;
  
  // no attributed
  behavior_.regionattrib = 0;

  // enclose in convex hull
  behavior_.convex = 1;
  
  // TODO: Figure out what weighted means
  behavior_.weighted = 0;
  // TODO: Figure out what jettison means
  behavior_.jettison = 0;

  // 0-based indexing
  behavior_.firstnumber = 0;

  // Voronoi disabled for now
  behavior_.voronoi = 0;

  // TODO: I dont think we care about triangle neighbors (maybe in the future for graph building?)
  behavior_.neighbors = 0;
  
  // TODO: Do we care about boundary information? I dont think so
  behavior_.nobound = 0;

  // no holes at the moment
  behavior_.noholes = 0;
  
  // not particularly concerned with exact arithmetic
  behavior_.noexact = 0;

  // dont want a true Delaunay triangulation
  behavior_.conformdel = 0;
  
  // use divide and conquer algorithm
  behavior_.incremental = 0;
  behavior_.sweepline = 0;

  // default for type of cuts
  behavior_.dwyer = 1;

  // dont want to split segments
  behavior_.splitseg = 0;

  // want to use segments
  behavior_.usesegments = 1;

  // dont want 6 points per triangle
  behavior_.order = 1;
  
  // TODO: Figure out if we want to supress boundary segment splitting
  behavior_.nobisect = 0;

  // dont insert Steiner points
  behavior_.steiner = -1;

  // no boundary on the minimum angle
  behavior_.minangle = 0.0;

  // Remaining:
  //  goodangle
  //  offconstant
  //  maxarea
  //  maxangle
  //  maxgoodangle

  //=====================Quick overrides=====================

  // Max area
  // behavior_.quality = 1;
  // behavior_.fixedarea = 1;
  // behavior_.maxarea = 1000.0;
  // behavior_.usesegments = 1;
  // behavior_.weighted = 0;

  // Conforming Delaunay
  // behavior_.quality = 1;
  // behavior_.conformdel = 1;

  // Minimum angle
  // behavior_.quality = 1;
  // behavior_.minangle = 20.0;
  // behavior_.goodangle = cos(behavior_.minangle * PI / 180.0);
  // if (behavior_.goodangle == 1.0) {
  //   behavior_.offconstant = 0.0;
  // } else {
  //   behavior_.offconstant = 0.475 * sqrt((1.0 + behavior_.goodangle) / (1.0 - behavior_.goodangle));
  // }
  // behavior_.goodangle *= behavior_.goodangle;
  // behavior_.usesegments = 1;
  // behavior_.weighted = 0;

}

triangle::behavior BehaviorBuilder::getBehavior() const {
  return behavior_;
}

void BehaviorBuilder::setConformingDelaunay(bool val) {
  if (val) {
    behavior_.quality = 1;
    behavior_.conformdel = 1;
  } else {
    behavior_.quality = 0;
    behavior_.conformdel = 0;
  }
}

void BehaviorBuilder::setEnforceMinimumAngle(bool val, float angle) {
  if (val) {
    behavior_.quality = 1;
    behavior_.minangle = angle;
    behavior_.goodangle = cos(behavior_.minangle * PI / 180.0);
    if (behavior_.goodangle == 1.0) {
      behavior_.offconstant = 0.0;
    } else {
      behavior_.offconstant = 0.475 * sqrt((1.0 + behavior_.goodangle) / (1.0 - behavior_.goodangle));
    }
    behavior_.goodangle *= behavior_.goodangle;
    behavior_.usesegments = 1;
    behavior_.weighted = 0;
  } else {
    behavior_.quality = 0;
  }
}

void BehaviorBuilder::setMinimumAngle(float angle) {
  behavior_.minangle = angle;
  behavior_.goodangle = cos(behavior_.minangle * PI / 180.0);
  if (behavior_.goodangle == 1.0) {
    behavior_.offconstant = 0.0;
  } else {
    behavior_.offconstant = 0.475 * sqrt((1.0 + behavior_.goodangle) / (1.0 - behavior_.goodangle));
  }
  behavior_.goodangle *= behavior_.goodangle;
}

void BehaviorBuilder::setEnforceMinimumArea(bool val, float area) {
  if (val) {
    behavior_.quality = 1;
    behavior_.fixedarea = 1;
    behavior_.maxarea = area;
    behavior_.usesegments = 1;
    behavior_.weighted = 0;
  // } else {
  //   behavior_.fixedarea = 0;
  }
}

void BehaviorBuilder::setMinimumArea(float area) {
  behavior_.maxarea = area;
}

} // namespace pathfinder
