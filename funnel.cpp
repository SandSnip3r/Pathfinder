#include "debuglogger.h"
#include "funnel.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <stdexcept>

BaseFunnel::BaseFunnel(const double agentRadius) : agentRadius_(agentRadius) {}

void BaseFunnel::funnelWithGoal(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint, const QPointF &goalPoint) {
  initializeForFunnelAlgorithm(corridor.size(), startPoint, &goalPoint);
  funnelForCorridor(corridor, startPoint);
  finishFunnelWithGoal(goalPoint);
}

void BaseFunnel::funnelWithoutGoal(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint) {
  initializeForFunnelAlgorithm(corridor.size(), startPoint);
  funnelForCorridor(corridor, startPoint);
}

QPointF BaseFunnel::finishFunnelAndFindClosestGoalOnEdge(const std::pair<QPointF,QPointF> &edge) {
    // Figure out which goal to use for the funnel
  const QPointF closestGoal = findBestGoalForFunnel(edge);
  finishFunnelWithGoal(closestGoal);
  return closestGoal;
}

void BaseFunnel::finishFunnelWithGoal(const QPointF &goalPoint) {
  // std::cout << "Adding goal (" << DebugLogger::instance().pointToString(goalPoint) << ") to funnel" << std::endl; //DEBUGPRINTS

  if (funnel_.point_in_funnel(goalPoint)) {
    // std::cout << "Goal already in funnel" << std::endl; //DEBUGPRINTS
    if (funnel_.front() == goalPoint) {
      // Goal is on the left, lets pop it and put it on the right
      // std::cout << "Goal is on the left of the funnel, removing it and putting it on the right" << std::endl; //DEBUGPRINTS
      if (funnel_.front() == funnel_.apex_point()) {
        throw std::runtime_error("Goal is the apex?");
      }
      funnel_.pop_front();
      addRight(goalPoint, true);
    } else {
      // TODO: Might need to handle cases where the goal is elsewhere in the funnel
      throw std::runtime_error("Goal is in funnel, but not at the left end");
    }
  } else {
    // Finally, add the goal to the right of the funnel
    // std::cout << "Goal not in funnel. Adding to the right" << std::endl; //DEBUGPRINTS
    addRight(goalPoint, true);
  }
  // std::cout << "Funnel after adding goal : "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
  // And finish the algorithm, closing out the funnel
  finishFunnel();
}

LengthFunnel LengthFunnel::cloneFunnelButSpaceFor1MorePoint() const {
  LengthFunnel newFunnel{*this};
  newFunnel.funnel_ = funnel_.cloneButSpaceFor1More();
  return newFunnel;
}

void BaseFunnel::initializeForFunnelAlgorithm(const int corridorSize, const QPointF &startPoint, const QPointF *goalPoint) {
  // Set up DebugLogger with some data
  DebugLogger::instance().setStartPoint(startPoint);
  if (goalPoint != nullptr) {
    DebugLogger::instance().setGoalPoint(*goalPoint);
  }

  if (corridorSize == 0) {
    throw std::runtime_error("Trying to funnel for an empty corridor");
  }

  // TODO: Consider better intialization technique
  funnel_ = Funnel(startPoint, corridorSize);
}

void BaseFunnel::extendByOneEdge(const std::pair<QPointF,QPointF> &edge) {
  // std::cout << "  Trying to extend funnel by edge {" << DebugLogger::instance().pointToString(edge.first) << ',' << DebugLogger::instance().pointToString(edge.second) << '}' << std::endl; //DEBUGPRINTS
  const auto &[vertexA, vertexB] = edge;
  if (vertexA == funnel_.back()) {
    addLeft(vertexB);
  } else if (vertexA == funnel_.front()) {
    addRight(vertexB);
  } else if (vertexB == funnel_.back()) {
    addLeft(vertexA);
  } else if (vertexB == funnel_.front()) {
    addRight(vertexA);
  } else {
    throw std::runtime_error("Unable to add edge to funnel because neither end matches the end of the funnel");
  }
}

void BaseFunnel::funnelForCorridor(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint) {
  // First and second point are the two ends of the first edge
  QPointF left, right;
  std::tie(left,right) = corridor.front();
  const bool areCorrectOrder = (math::crossProduct(startPoint, right, startPoint, left) > 0);
  if (!areCorrectOrder) {
    // incorrect order (left is actually right), swap
    std::swap(left, right);
  }
  // std::cout << "=============================================" << std::endl; //DEBUGPRINTS
  // std::cout << "Starting funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS

  addLeft(left);
  // std::cout << "Funnel after adding left: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS

  addRight(right);
  // std::cout << "Funnel before loop (after adding right): "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS

  for (int i=1; i<corridor.size(); ++i) {
    extendByOneEdge(corridor.at(i));
    // std::cout << "Funnel : "; //DEBUGPRINTS
    // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
  }

  // std::cout << "Funnel after loop : "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
}

void BaseFunnel::addLeft(const QPointF &point, const bool isGoal) {
  addPointToFunnel(std::bind(&Funnel::begin, &funnel_),
                   std::bind(&Funnel::end, &funnel_),
                   std::bind(&Funnel::apex_it, &funnel_),
                   std::bind(&Funnel::apex_type, &funnel_),
                   [this](){ return funnel_.set_apex_index(0); },
                   std::bind(&Funnel::set_apex_type, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::push_front, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::push_front_apex, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::pop_front, &funnel_),
                   AngleDirection::kCounterclockwise,
                   point,
                   isGoal);
}

void BaseFunnel::addRight(const QPointF &point, const bool isGoal) {
  addPointToFunnel(std::bind(&Funnel::rbegin, &funnel_),
                   std::bind(&Funnel::rend, &funnel_),
                   std::bind(&Funnel::rapex_it, &funnel_),
                   std::bind(&Funnel::apex_type, &funnel_),
                   [this](){ return funnel_.set_apex_index(funnel_.size()-1); },
                   std::bind(&Funnel::set_apex_type, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::push_back, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::push_back_apex, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::pop_back, &funnel_),
                   AngleDirection::kClockwise,
                   point,
                   isGoal);
}

void BaseFunnel::finishFunnel() {
  // NOTE: Assumes that the goal was previously added to the right of the funnel
  AngleDirection t1 = funnel_.apex_type();
  AngleDirection t2 = AngleDirection::kClockwise;
  for (int i=funnel_.apex_index(); i<funnel_.size()-1;) {
    if (i+1 == funnel_.size()-1) {
      // Next point is the end of the last right edge
      t2 = AngleDirection::kPoint;
    }
    std::pair<QPointF, QPointF> newEdge = math::createCircleConsciousLine(funnel_.at(i), t1, funnel_.at(i+1), t2, agentRadius_);
    addSegment(Apex{funnel_.at(i), t1}, newEdge, Apex{funnel_.at(i+1), t2});
    t1 = AngleDirection::kClockwise;
    i += 1;
  }
}

void PathFunnel::addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) {
  // std::cout << "  ~Adding segment to path for PathFunnel" << std::endl; //DEBUGPRINTS
  // First, finish an arc if there is an open one
  if (previousApex.apexType != AngleDirection::kPoint && agentRadius_ > 0.0) {
    // Finish previously created arc
    //  We know that there must be at least one arc in the path
    PathSegment *mostRecentSegment = path_.back().get();
    ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(mostRecentSegment);
    if (arc == nullptr) {
      throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
    }
    // A couple sanity checks, TODO: Remove after
    if (arc->circleCenter != previousApex.apexPoint) {
      throw std::runtime_error("arc->circleCenter != previousApex.apexPoint");
    } else if (arc->angleDirection != previousApex.apexType) {
      throw std::runtime_error("arc->angleDirection != previousApex.apexType");
    }
    arc->endAngle = math::angle(previousApex.apexPoint, edge.first);
    // std::cout << "    ~Finishing arc (angle: " << arc->endAngle << ")" << std::endl; //DEBUGPRINTS
  }

  // Second, add a straight segment between the apexes
  // std::cout << "    ~Adding straight segment from " << DebugLogger::instance().pointToString(edge.first) << " to " << DebugLogger::instance().pointToString(edge.second) << std::endl; //DEBUGPRINTS
  path_.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(edge.first, edge.second)));

  if (newApex.apexType != AngleDirection::kPoint && agentRadius_ > 0.0) {
    // Finally, since this isn't the end, start a new arc
    path_.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(newApex.apexPoint, agentRadius_, newApex.apexType)));
    PathSegment *mostRecentSegment = path_.back().get();
    ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(mostRecentSegment);
    if (arc == nullptr) {
      throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
    }
    arc->startAngle = math::angle(newApex.apexPoint, edge.second);
    // std::cout << "    ~Not the end, adding the start of another arc (angle: " << arc->startAngle << ")" << std::endl; //DEBUGPRINTS
  }
}

PathFunnel::PathType PathFunnel::getPath() const {
  PathType result;
  result.reserve(path_.size());
  for (const auto &i : path_) {
    result.emplace_back(i->clone());
  }
  return result;
}

double PathFunnel::currentPathLength() const {
  // TODO:
  // // std::cout << "    calculatePathLength: "; //DEBUGPRINTS
  // double totalDistance=0;
  // for (int i=0; i<path.size(); ++i) {
  //   const PathSegment *segment = path.at(i).get();
  //   const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(segment);
  //   const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(segment);
  //   if (straightSegment != nullptr) {
  //     totalDistance += math::distance(straightSegment->startPoint, straightSegment->endPoint);
  //     // std::cout << "+straight [" << totalDistance << "], "; //DEBUGPRINTS
  //   } else if (arcSegment != nullptr) {
  //     if (i != path.size()-1) {
  //       // If the last segment is an arc, i think its probably an unfinished path and we shouldnt add this segment
  //       double angle = math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, (arcSegment->angleDirection == AngleDirection::kCounterclockwise));
  //       totalDistance += arcSegment->circleRadius * abs(angle);
  //       // std::cout << "+arc (angle start:" << arcSegment->startAngle << ", angle end: " << arcSegment->endAngle << ", angle: " << angle << ", radius:" << arcSegment->circleRadius << ") [" << totalDistance << "], "; //DEBUGPRINTS
  //     }
  //   }
  // }
  // // std::cout << std::endl; //DEBUGPRINTS
  // return totalDistance;
  throw std::runtime_error("PathFunnel::currentPathLength() Not implemented");
}

QPointF PathFunnel::findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const {
  throw std::runtime_error("PathFunnel::findBestGoalForFunnel() Not implemented");
}

void LengthFunnel::addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) {
  // std::cout << "  ~Adding segment to path for LengthFunnel" << std::endl; //DEBUGPRINTS
  // First, finish an arc if there is an open one
  if (previousApex.apexType != AngleDirection::kPoint && agentRadius_ > 0.0) {
    // An arc has been started
    if (!previousAngle_) {
      throw std::runtime_error("There should be an angle for the start of this arc");
    }
    const auto currentAngle = math::angle(previousApex.apexPoint, edge.first);
    double angle = math::arcAngle(*previousAngle_, currentAngle, previousApex.apexType);
    // std::cout << "    ~Adding arc length (angle: " << angle << ", length:" << agentRadius_ * abs(angle) << ")" << std::endl; //DEBUGPRINTS
    length_ += agentRadius_ * abs(angle);

    // Throw away stored angle
    previousAngle_.reset();
  }

  // Second, add a straight segment between the apexes
  // std::cout << "    ~Adding straight segment from " << DebugLogger::instance().pointToString(edge.first) << " to " << DebugLogger::instance().pointToString(edge.second) << ", length: " << math::distance(edge.first, edge.second) << std::endl; //DEBUGPRINTS
  length_ += math::distance(edge.first, edge.second);

  if (newApex.apexType != AngleDirection::kPoint && agentRadius_ > 0.0) {
    // Finally, since this isn't the end, start a new arc
    previousAngle_ = math::angle(newApex.apexPoint, edge.second);
    // std::cout << "    ~Not the end, adding the start of another arc (angle: " << *previousAngle_ << ")" << std::endl; //DEBUGPRINTS
  }
}

double LengthFunnel::getLength() const {
  return length_;
}

double LengthFunnel::funnelLengthForAgentWithRadius(LengthFunnel funnelCopy, const QPointF &goalPoint) const {
  // Do not work with data of self, work with data of copy
  // std::cout << "  ::Checking funnel length from potential apex" << std::endl; //DEBUGPRINTS

  if (funnelCopy.funnel_.point_in_funnel(goalPoint)) {
    if (funnelCopy.funnel_.front() == goalPoint) {
      // Goal is on the left, lets pop it and put it on the right
      if (funnelCopy.funnel_.front() == funnelCopy.funnel_.apex_point()) {
        throw std::runtime_error("Goal is the apex?");
      }
      funnelCopy.funnel_.pop_front();
      funnelCopy.addRight(goalPoint, true);
    } else if (funnelCopy.funnel_.back() == goalPoint) {
      // std::cout << "    ::Goal point is already in the right spot in the funnel" << std::endl; //DEBUGPRINTS
    } else {
      // TODO: Handle
      // std::cout << "    ::Goal point is in the funnel, but not at the right or left" << std::endl; //DEBUGPRINTS
      throw std::runtime_error("Goal is in funnel, but not at the left end");
    }
  } else {
    // Finally, add the goal to the right of the funnel
    funnelCopy.addRight(goalPoint, true);
  }
  // And finish the algorithm, closing out the funnel
  funnelCopy.finishFunnel();

  // std::cout << "    ::Completed funnel : "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnelCopy); //DEBUGPRINTS

  return funnelCopy.getLength();
}

QPointF LengthFunnel::findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const {
  QPointF result;

  // std::cout << "  Need to find goal for funnel" << std::endl; //DEBUGPRINTS
  // No goal given, this must mean we're funneling inside of the A* algorithm
  // We dont know where we want the goal point of the funnel to be
  //  It should be the closest point to the last apex of the funnel
  //  However, the apex depends on the goal point
  // Any point on the left or right of the funnel could end up being the apex. We need to check all of them and use the one that results in the shortest path
  if (funnel_.empty()) {
    throw std::runtime_error("Funnel is empty");
  }

  // Save the length of the current path
  const double pathLengthThusFar = currentPathLength();

  // Track the best option
  double shortestPathLength = std::numeric_limits<double>::max();

  // Save the final edge, we will reference it many times
  const auto [edgeStart, edgeEnd] = lastEdgeOfCorridor;

  // For each point in the funnel, find the closest point on the target edge, then check what the overall funnel length is
  AngleDirection funnelApexAngleDirection = AngleDirection::kCounterclockwise;
  for (int funnelIndex=0; funnelIndex<funnel_.size(); ++funnelIndex) {
    const QPointF &currentFunnelPoint = funnel_.at(funnelIndex);
    if (currentFunnelPoint == funnel_.apex_point()) {
      // We've reached the apex of the funnel, there are no more points left on the left of the funnel
      funnelApexAngleDirection = funnel_.apex_type();
    }
    // Test that, if this was the final apex, would it result in the shortest path
    QPointF potentialGoal;
    if (currentFunnelPoint == edgeStart) {
      // This point of the funnel is the start of the edge, need to move it over by the character radius
      // std::cout << "  This point of the funnel (" << DebugLogger::instance().pointToString(currentFunnelPoint) << ") is the start of the edge. Need to move" << std::endl; //DEBUGPRINTS
      potentialGoal = edgeStart;
    } else if (currentFunnelPoint == edgeEnd) {
      // This point of the funnel is the end of the edge, need to move it over by the character radius
      // std::cout << "  This point of the funnel (" << DebugLogger::instance().pointToString(currentFunnelPoint) << ") is the end of the edge. Need to move" << std::endl; //DEBUGPRINTS
      potentialGoal = edgeEnd;
    } else {
      // This point of the funnel is not one of the ends of the edge
      // std::cout << "  Checking distance from a point on the funnel (" << DebugLogger::instance().pointToString(currentFunnelPoint) << ") to the target edge (" << DebugLogger::instance().pointToString(edgeStart) << "->" << DebugLogger::instance().pointToString(edgeEnd) << ")" << std::endl; //DEBUGPRINTS
      math::distanceBetweenEdgeAndCircleTangentIntersectionPoint(edgeStart, edgeEnd, currentFunnelPoint, agentRadius_, funnelApexAngleDirection, &potentialGoal);
    }
    bool moved = false;
    if (agentRadius_ > 0 && math::distance(edgeStart, potentialGoal) < agentRadius_) {
      // Need to move the point towards the other end of the edge
      // std::cout << "  Moving point towards edgeEnd" << std::endl; //DEBUGPRINTS
      const double edgeDx = edgeEnd.x()-edgeStart.x();
      const double edgeDy = edgeEnd.y()-edgeStart.y();
      const double edgeLength = sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
      const double ratio = agentRadius_/edgeLength;
      potentialGoal = QPointF{edgeStart.x()+edgeDx*ratio, edgeStart.y()+edgeDy*ratio};
      moved = true;
    } else if (agentRadius_ > 0 && math::distance(edgeEnd, potentialGoal) < agentRadius_) {
      // Need to move the point towards the other end of the edge
      // std::cout << "  Moving point towards edgeStart" << std::endl; //DEBUGPRINTS
      const double edgeDx = edgeStart.x()-edgeEnd.x();
      const double edgeDy = edgeStart.y()-edgeEnd.y();
      const double edgeLength = sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
      const double ratio = agentRadius_/edgeLength;
      potentialGoal = QPointF{edgeEnd.x()+edgeDx*ratio, edgeEnd.y()+edgeDy*ratio};
      moved = true;
    }
    if (moved) {
      // std::cout << "  Moved point result: " << DebugLogger::instance().pointToString(potentialGoal) << std::endl; //DEBUGPRINTS
    }

    const double remainingPathLength = funnelLengthForAgentWithRadius(*this, potentialGoal);
    const double pathLength = pathLengthThusFar + remainingPathLength;
    if (pathLength < shortestPathLength) {
      shortestPathLength = pathLength;
      result = potentialGoal;
      // std::cout << "  New shortest path (len=" << shortestPathLength << ")" << std::endl; //DEBUGPRINTS
    }

    if (currentFunnelPoint == funnel_.apex_point()) {
      // Now we're moving on to the right of the funnel
      funnelApexAngleDirection = AngleDirection::kClockwise;
    }
  }
  // std::cout << "Funnel ends with a segment from apex " << DebugLogger::instance().pointToString(funnel.apex_point()) << " to " << DebugLogger::instance().pointToString(goalPointToUse) << std::endl; //DEBUGPRINTS

  return result;
}

double LengthFunnel::currentPathLength() const {
  return length_;
}

// ===================================================================================
// ============================Funnel data structure below============================
// ===================================================================================

Funnel::Funnel(const QPointF &initialApex, const int corridorSize) : funnel_(1 + corridorSize*2 + 2) {
  // Allocate enough space for the entire corridor to fit on either side, +1 for the apex, and +2 for the possibility of the goal on either side
  apexIndex_ = funnel_.size()/2;
  funnel_.at(apexIndex_) = initialApex;
  leftIndex_ = apexIndex_;
  rightIndex_ = apexIndex_;
}

int Funnel::size() const {
  return (rightIndex_-leftIndex_) + 1;
}

bool Funnel::empty() const {
  if (leftIndex_ > rightIndex_) {
    throw std::runtime_error("Funnel is empty. Shouldnt be possible");
  }
  return false;
}

// Funnel::const_iterator Funnel::begin() const {
//   return funnel_.begin() + leftIndex_;
// }

// Funnel::const_iterator Funnel::end() const {
//   return funnel_.begin() + leftIndex_ + size();
// }

// Funnel::const_reverse_iterator Funnel::rbegin() const {
//   return funnel_.rbegin() + (size()-1-rightIndex_);
// }

// Funnel::const_reverse_iterator Funnel::rend() const {
//   return funnel_.rbegin() + (size()-1-rightIndex_) + size();
// }

Funnel::iterator Funnel::begin() {
  return funnel_.begin() + leftIndex_;
}

Funnel::iterator Funnel::end() {
  return begin() + size();
}

Funnel::iterator Funnel::apex_it() {
  return funnel_.begin() + apexIndex_;
}

Funnel::reverse_iterator Funnel::rbegin() {
  return funnel_.rbegin() + (funnel_.size()-1-rightIndex_);
}

Funnel::reverse_iterator Funnel::rend() {
  return rbegin() + size();
}

Funnel::reverse_iterator Funnel::rapex_it() {
  return funnel_.rbegin() + (funnel_.size()-1-apexIndex_);
}

const QPointF& Funnel::at(int index) const {
  if (index >= size()) {
    throw std::runtime_error("Accessing funnel out of bounds");
  }
  return funnel_.at(leftIndex_+index);
}

const QPointF& Funnel::front() const {
  return funnel_.at(leftIndex_);
}

void Funnel::pop_front() {
  ++leftIndex_;
}

void Funnel::push_front(const QPointF &point) {
  --leftIndex_;
  funnel_.at(leftIndex_) = point;
}

const QPointF& Funnel::back() const {
  return funnel_.at(rightIndex_);
}

void Funnel::pop_back() {
  --rightIndex_;
}

void Funnel::push_back(const QPointF &point) {
  ++rightIndex_;
  funnel_.at(rightIndex_) = point;
}

bool Funnel::point_in_funnel(const QPointF &point) const {
  return std::find(funnel_.begin()+leftIndex_, (funnel_.begin()+leftIndex_+size()), point) != (funnel_.begin()+leftIndex_+size());
}

Apex Funnel::funnel_apex() const {
  return Apex{apex_point(), apex_type()};
}

const QPointF& Funnel::apex_point() const {
  return funnel_.at(apexIndex_);
}

AngleDirection Funnel::apex_type() const {
  return apexType_;
}

int Funnel::apex_index() const {
  return apexIndex_-leftIndex_;
}

void Funnel::push_front_apex(const Apex &apex) {
  push_front(apex.apexPoint);
  apexIndex_ = leftIndex_;
  apexType_ = apex.apexType;
}

void Funnel::push_back_apex(const Apex &apex) {
  push_back(apex.apexPoint);
  apexIndex_ = rightIndex_;
  apexType_ = apex.apexType;
}

void Funnel::set_apex_index(const int index) {
  apexIndex_ = leftIndex_+index;
}

void Funnel::set_apex_type(const AngleDirection type) {
  apexType_ = type;
}

Funnel Funnel::cloneButSpaceFor1More() const {
  // Assuming that we havent already added the goal, we already must have one free space on each side
  //  However, we now need to make sure that we have two free spaces on each side, one for the new edge and one more for the goal
  if (rightIndex_ >= funnel_.size()-2 || leftIndex_ < 2) {
    // Not enough room in funnel for one more edge and goal, need to allocate a larger one
    Funnel newFunnel;
    // Allocate enough space for one more point on each side
    newFunnel.funnel_.resize(funnel_.size()+2);
    // Copy all points to new funnel (shifted to the right by 1)
    std::copy(funnel_.begin()+leftIndex_, funnel_.begin()+leftIndex_+size(), newFunnel.funnel_.begin()+leftIndex_+1);
    // Copy other data (shifted to the right by 1)
    newFunnel.apexType_ = apexType_;
    newFunnel.apexIndex_ = apexIndex_+1;
    newFunnel.leftIndex_ = leftIndex_+1;
    newFunnel.rightIndex_ = rightIndex_+1;
    return newFunnel;
  } else {
    // Already enough room in funnel, just return a copy of this one
    return *this;
  }
}