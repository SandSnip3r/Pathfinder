#include "debuglogger.h"
#include "funnel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace pathfinder {

BaseFunnel::BaseFunnel(const double agentRadius) : agentRadius_(agentRadius) {}

void BaseFunnel::funnelWithGoal(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint, const Vector &goalPoint) {
  // std::cout << ">>>>BaseFunnel::funnelWithGoal" << std::endl; //DEBUGPRINTS
  initializeForFunnelAlgorithm(corridor.size(), startPoint, &goalPoint);
  funnelForCorridor(corridor, startPoint);
  finishFunnelWithGoal(goalPoint);
}

void BaseFunnel::funnelWithoutGoal(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint) {
  // std::cout << ">>>>BaseFunnel::funnelWithoutGoal" << std::endl; //DEBUGPRINTS
  initializeForFunnelAlgorithm(corridor.size(), startPoint);
  funnelForCorridor(corridor, startPoint);
}

Vector BaseFunnel::finishFunnelAndFindClosestGoalOnEdge(const std::pair<Vector,Vector> &edge) {
  // Figure out which goal to use for the funnel
  const Vector closestGoal = findBestGoalForFunnel(edge);
  finishFunnelWithGoal(closestGoal);
  return closestGoal;
}

void BaseFunnel::finishFunnelWithGoal(const Vector &goalPoint) {
  // std::cout << "Adding goal (" << DebugLogger::instance().pointToString(goalPoint) << ") to funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS

  if (funnel_.point_in_funnel(goalPoint)) {
    // std::cout << "Goal already in funnel" << std::endl; //DEBUGPRINTS
    if (funnel_.apex_point() == goalPoint) {
      // The goal is already the apex
      // std::cout << "Goal is the apex. Funnel : "; //DEBUGPRINTS
      // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
      // The funnel is done at this point. We dont even call finishFunnel()
      return;
    }
    if (funnel_.front() == goalPoint) {
      // Goal is on the left, lets pop it and put it on the right
      // std::cout << "Goal is on the left of the funnel, removing it and putting it on the right" << std::endl; //DEBUGPRINTS
      funnel_.pop_front();
      addRight(goalPoint, true);
    } else if (funnel_.back() == goalPoint) {
      // Goal is already on the right, nothing to do
      // std::cout << "Goal is already on the right, nothing to do" << std::endl; //DEBUGPRINTS
    } else {
      // TODO: Might need to handle cases where the goal is elsewhere in the funnel
      // std::cout << "Goal is in funnel, but not at the either end. Funnel : "; //DEBUGPRINTS
      // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
      throw std::runtime_error("Goal is in funnel, but not at either end");
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

void BaseFunnel::initializeForFunnelAlgorithm(const std::size_t corridorSize, const Vector &startPoint, const Vector *goalPoint) {
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

void BaseFunnel::extendByOneEdge(const std::pair<Vector,Vector> &edge) {
  // std::cout << "  Trying to extend funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
  // std::cout << "    by edge {" << DebugLogger::instance().pointToString(edge.first) << ',' << DebugLogger::instance().pointToString(edge.second) << '}' << std::endl; //DEBUGPRINTS
  // TODO: Not sure why the point of the given edge isnt more closely matching what exists in the funnel
  // TODO: Would it make sense to move the point in the funnel to match the given edge?
  constexpr const double kReducedPrecision{0.00153}; // TODO: Remove once we are confident that the below code wont throw
  const auto &[vertexA, vertexB] = edge;
  const auto distanceAToFront = math::distance(vertexA, funnel_.front());
  const auto distanceBToBack = math::distance(vertexB, funnel_.back());
  const auto distanceAToBack = math::distance(vertexA, funnel_.back());
  const auto distanceBToFront = math::distance(vertexB, funnel_.front());
  if ((distanceAToFront+distanceBToBack) < (distanceAToBack+distanceBToFront)) {
    if (distanceAToFront < distanceBToBack) {
      // A is likely to be the same point as front()
      if (math::equal(vertexA, funnel_.front())) {
        addRight(vertexB);
      } else if (math::equal(vertexA, funnel_.front(), kReducedPrecision)) {
        const auto distanceBetweenPoints = math::distance(vertexA, funnel_.front());
        throw std::runtime_error("Some precision issues when we expected vertexA to be the same point as front() "+std::to_string(distanceBetweenPoints));
        addRight(vertexB);
      } else {
        throw std::runtime_error("Unable to add edge to funnel because neither end matches the end of the funnel");
      }
    } else {
      // B is likely to be the same point as back()
      if (math::equal(vertexB, funnel_.back())) {
        addLeft(vertexA);
      } else if (math::equal(vertexB, funnel_.back(), kReducedPrecision)) {
        const auto distanceBetweenPoints = math::distance(vertexB, funnel_.back());
        throw std::runtime_error("Some precision issues when we expected vertexB to be the same point as back() "+std::to_string(distanceBetweenPoints));
        addLeft(vertexA);
      } else {
        throw std::runtime_error("Unable to add edge to funnel because neither end matches the end of the funnel");
      }
    }
  } else {
    if (distanceAToBack < distanceBToFront) {
      // A is likely to be the same point as back()
      if (math::equal(vertexA, funnel_.back())) {
        addLeft(vertexB);
      } else if (math::equal(vertexA, funnel_.back(), kReducedPrecision)) {
        const auto distanceBetweenPoints = math::distance(vertexA, funnel_.back());
        throw std::runtime_error("Some precision issues when we expected vertexA to be the same point as back() "+std::to_string(distanceBetweenPoints));
        addLeft(vertexB);
      } else {
        throw std::runtime_error("Unable to add edge to funnel because neither end matches the end of the funnel");
      }
    } else {
      // B is likely to be the same point as front()
      if (math::equal(vertexB, funnel_.front())) {
        addRight(vertexA);
      } else if (math::equal(vertexB, funnel_.front(), kReducedPrecision)) {
        const auto distanceBetweenPoints = math::distance(vertexB, funnel_.front());
        throw std::runtime_error("Some precision issues when we expected vertexB to be the same point as front() "+std::to_string(distanceBetweenPoints));
        addRight(vertexA);
      } else {
        throw std::runtime_error("Unable to add edge to funnel because neither end matches the end of the funnel");
      }
    }
  }
  // std::cout << "  Edge added, resulting funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
}

void BaseFunnel::funnelForCorridor(const std::vector<std::pair<Vector,Vector>> &corridor, const Vector &startPoint) {
  // First and second point are the two ends of the first edge
  Vector left, right;
  std::tie(left,right) = corridor.front();
  const bool areCorrectOrder = (math::crossProductForSign(startPoint, right, startPoint, left) > 0);
  if (!areCorrectOrder) {
    // incorrect order (left is actually right), swap
    std::swap(left, right);
  }
  // std::cout << "==========BaseFunnel::funnelForCorridor" << std::endl; //DEBUGPRINTS
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

bool BaseFunnel::pointInFunnel(const Vector &point) const {
  for (int i=0; i<funnel_.size(); ++i) {
    if (math::equal(funnel_.at(i), point)) {
      return true;
    }
  }
  return false;
}

void BaseFunnel::addLeft(const Vector &point, const bool isGoal) {
  if (pointInFunnel(point)) {
    throw std::runtime_error("Trying to add a point to the funnel which is already in the funnel");
  }
  addPointToFunnel(std::bind(&Funnel::at, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::apex_index, &funnel_),
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

void BaseFunnel::addRight(const Vector &point, const bool isGoal) {
  if (pointInFunnel(point)) {
    throw std::runtime_error("Trying to add a point to the funnel which is already in the funnel");
  }
  addPointToFunnel(std::bind(&Funnel::reverse_at, &funnel_, std::placeholders::_1),
                   std::bind(&Funnel::reverse_apex_index, &funnel_),
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
  // std::cout << "Finishing funnel" << std::endl; //DEBUGPRINTS
  AngleDirection t1 = funnel_.apex_type();
  AngleDirection t2 = AngleDirection::kClockwise;
  for (int i=funnel_.apex_index(); i<funnel_.size()-1;) {
    if (i+1 == funnel_.size()-1) {
      // Next point is the end of the last right edge
      t2 = AngleDirection::kNoDirection;
    }
    // std::cout << "newWedge is " << DebugLogger::instance().pointToString(funnel_.at(i))   << ' ' << (t1 == AngleDirection::kNoDirection ? " point" : (t1 == AngleDirection::kClockwise ? " right" : " left")) << " -> " //DEBUGPRINTS
                                // << DebugLogger::instance().pointToString(funnel_.at(i+1)) << ' ' << (t2 == AngleDirection::kNoDirection ? " point" : (t2 == AngleDirection::kClockwise ? " right" : " left")) << std::endl; //DEBUGPRINTS
    std::pair<Vector, Vector> newEdge = math::createCircleConsciousLine(funnel_.at(i), t1, funnel_.at(i+1), t2, agentRadius_);
    // std::cout << "  New edge: " << newEdge.first.x()  << ',' << newEdge.first.y() << "->" //DEBUGPRINTS
                                // << newEdge.second.x() << ',' << newEdge.second.y() << std::endl; //DEBUGPRINTS
    addSegment(Apex{funnel_.at(i), t1}, newEdge, Apex{funnel_.at(i+1), t2});
    t1 = AngleDirection::kClockwise;
    i += 1;
  }
}

void PathFunnel::addSegment(const Apex &previousApex, const std::pair<Vector,Vector> &edge, const Apex &newApex) {
  // std::cout << "  ~Adding segment to path for PathFunnel" << std::endl; //DEBUGPRINTS
  // First, finish an arc if there is an open one
  if (previousApex.apexType != AngleDirection::kNoDirection && agentRadius_ > 0.0) {
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
    // std::cout << "    ~Finishing arc (start angle: " << arc->startAngle << ", end angle: " << arc->endAngle << ")" << std::endl; //DEBUGPRINTS
  }

  // Second, add a straight segment between the apexes
  const auto segmentLength = math::distance(edge.first, edge.second);
  // Ignore segments with length 0
  if (!math::equal(segmentLength, 0)) {
    // std::cout << "    ~Adding straight segment (length = " << segmentLength << ") from " << DebugLogger::instance().pointToString(edge.first) << " to " << DebugLogger::instance().pointToString(edge.second) << std::endl; //DEBUGPRINTS
    path_.emplace_back(std::unique_ptr<PathSegment>(new StraightPathSegment(edge.first, edge.second)));
  }

  if (newApex.apexType != AngleDirection::kNoDirection && agentRadius_ > 0.0) {
    // Finally, since this isn't the end, start a new arc
    path_.emplace_back(std::unique_ptr<PathSegment>(new ArcPathSegment(newApex.apexPoint, agentRadius_, newApex.apexType)));
    PathSegment *mostRecentSegment = path_.back().get();
    ArcPathSegment *arc = dynamic_cast<ArcPathSegment*>(mostRecentSegment);
    if (arc == nullptr) {
      throw std::runtime_error("Last element of path isnt an ArcPathSegment?");
    }
    arc->startAngle = math::angle(newApex.apexPoint, edge.second);
    // std::cout << "    ~Not the end, adding the start of another arc (angle: " << arc->startAngle << ")" << std::endl; //DEBUGPRINTS
    // std::cout << "    ~Based on edge.second: " << edge.second.x() << ',' << edge.second.y() << std::endl; //DEBUGPRINTS
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

Vector PathFunnel::findBestGoalForFunnel(const std::pair<Vector,Vector> &lastEdgeOfCorridor) const {
  throw std::runtime_error("PathFunnel::findBestGoalForFunnel() Not implemented");
}

void LengthFunnel::addSegment(const Apex &previousApex, const std::pair<Vector,Vector> &edge, const Apex &newApex) {
  // std::cout << "  ~Adding segment to path for LengthFunnel" << std::endl; //DEBUGPRINTS
  // First, finish an arc if there is an open one
  if (previousApex.apexType != AngleDirection::kNoDirection && agentRadius_ > 0.0) {
    // An arc has been started
    if (!previousAngle_) {
      throw std::runtime_error("There should be an angle for the start of this arc");
    }
    const auto currentAngle = math::angle(previousApex.apexPoint, edge.first);
    double angle = math::arcAngle(*previousAngle_, currentAngle, previousApex.apexType);
    // std::cout << "    ~Adding arc length (angle: " << angle << ", length:" << agentRadius_ * abs(angle) << ")" << std::endl; //DEBUGPRINTS
    length_ += agentRadius_ * std::abs(angle);

    // Throw away stored angle
    previousAngle_.reset();
  }

  // Second, add a straight segment between the apexes
  // std::cout << "    ~Adding straight segment from " << DebugLogger::instance().pointToString(edge.first) << " to " << DebugLogger::instance().pointToString(edge.second) << ", length: " << math::distance(edge.first, edge.second) << std::endl; //DEBUGPRINTS
  length_ += math::distance(edge.first, edge.second);

  if (newApex.apexType != AngleDirection::kNoDirection && agentRadius_ > 0.0) {
    // Finally, since this isn't the end, start a new arc
    previousAngle_ = math::angle(newApex.apexPoint, edge.second);
    // std::cout << "    ~Not the end, adding the start of another arc (angle: " << *previousAngle_ << ")" << std::endl; //DEBUGPRINTS
  }
}

double LengthFunnel::getLength() const {
  return length_;
}

double LengthFunnel::funnelLengthForAgentWithRadius(LengthFunnel funnelCopy, const Vector &goalPoint) const {
  funnelCopy.finishFunnelWithGoal(goalPoint);
  return funnelCopy.getLength();
}

Vector LengthFunnel::findBestGoalForFunnel(const std::pair<Vector,Vector> &lastEdgeOfCorridor) const {
  constexpr const double kReducedPrecision{0.00153};
  std::optional<Vector> result;

  // std::cout << "  Need to find goal for funnel: "; //DEBUGPRINTS
  // DebugLogger::instance().printFunnel(funnel_); //DEBUGPRINTS
  // std::cout << "  Final edge: " << DebugLogger::instance().pointToString(lastEdgeOfCorridor.first) << ',' << DebugLogger::instance().pointToString(lastEdgeOfCorridor.second) << std::endl; //DEBUGPRINTS
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
  Vector edgeStart = lastEdgeOfCorridor.first;
  Vector edgeEnd = lastEdgeOfCorridor.second;

  if (edgeStart == funnel_.apex_point() || edgeEnd == funnel_.apex_point()) {
    // Apex is currently on the goal edge
    // std::cout << "Apex is currently on the goal edge" << std::endl; //DEBUGPRINTS
    return funnel_.apex_point();
  }

  if (funnel_.size() < 2) {
    throw std::runtime_error("Funnel too small to find a goal");
  }
  if (math::equal(edgeEnd, funnel_.front(), kReducedPrecision) && math::equal(edgeStart, funnel_.back(), kReducedPrecision)) {
    if (!(math::equal(edgeEnd, funnel_.front()) && math::equal(edgeStart, funnel_.back()))) {
      throw std::runtime_error("Precision issues in findBestGoalForFunnel");
    }
    // Opposite orientation that we are expecting, flip it
    // Expect funnel.front() (left) to be the start of the edge and funnel.back() to be the end of the edge
    std::swap(edgeStart, edgeEnd);
  } else if (!math::equal(edgeStart, funnel_.front(), kReducedPrecision) || !math::equal(edgeEnd, funnel_.back(), kReducedPrecision)) {
    if (!(!math::equal(edgeStart, funnel_.front()) || !math::equal(edgeEnd, funnel_.back()))) {
      throw std::runtime_error("Precision issues in findBestGoalForFunnel");
    }
    throw std::runtime_error("Ends of the funnel are not the target edge!");
  }

  if (!math::equal(agentRadius_, 0.0)) {
    // Lets trim the final edge so that no invalid points can be chosen
    // TODO: This algorithm would change once non-constraint vertices are allowed
    {
      // Move start of edge over
      const double edgeDx = edgeEnd.x()-edgeStart.x();
      const double edgeDy = edgeEnd.y()-edgeStart.y();
      const double edgeLength = std::sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
      const double ratio = agentRadius_/edgeLength;
      edgeStart.setX(edgeStart.x()+edgeDx*ratio);
      edgeStart.setY(edgeStart.y()+edgeDy*ratio);
    }
    {
      // Move end of edge over
      const double edgeDx = edgeStart.x()-edgeEnd.x();
      const double edgeDy = edgeStart.y()-edgeEnd.y();
      const double edgeLength = std::sqrt(edgeDx*edgeDx + edgeDy*edgeDy);
      const double ratio = agentRadius_/edgeLength;
      edgeEnd.setX(edgeEnd.x()+edgeDx*ratio);
      edgeEnd.setY(edgeEnd.y()+edgeDy*ratio);
    }

    // We know that the first and last point of the funnel are the start and end of the final edge, no point in checking
    for (int tmpFunnelIndex=1; tmpFunnelIndex<funnel_.apex_index(); ++tmpFunnelIndex) {
      const Vector &tmpFunnelPoint = funnel_.at(tmpFunnelIndex);
      Vector intersectionPoint1, intersectionPoint2;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(edgeStart, edgeEnd, tmpFunnelPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
      if (intersectionCount > 0) {
        // Intersects with a point of another vertex, need to move it further
        if (intersectionCount == 1) {
          if (math::lessThan(math::distance(intersectionPoint1, edgeEnd), math::distance(edgeStart, edgeEnd))) {
            // New point is closer to the end, need to move
            edgeStart = intersectionPoint1;
          }
        } else {
          // Must be 2 intersections, choose the one even further from the start
          if (math::lessThan(math::distance(intersectionPoint1, edgeEnd), math::distance(intersectionPoint2, edgeEnd))) {
            edgeStart = intersectionPoint1;
          } else {
            edgeStart = intersectionPoint2;
          }
        }
      }
    }
    {
      if (funnel_.apex_type() != AngleDirection::kNoDirection) {
        // If the apex type is "kNoDirection" then that means its the start point of the algorithm and we dont care about it intersecting with the goal edge
        Vector intersectionPoint1, intersectionPoint2;
        const int intersectionCount = math::lineSegmentIntersectsWithCircle(edgeStart, edgeEnd, funnel_.apex_point(), agentRadius_, &intersectionPoint1, &intersectionPoint2);
        if (intersectionCount > 0) {
          // Depending on the direction of rotation around this apex, either throw away the left or the right of the goal edge
          // If clockwise, throw away right (move end)
          Vector &pointToChange = (funnel_.apex_type() == AngleDirection::kClockwise ? edgeEnd : edgeStart);
          // Intersects with a point of another vertex, need to move it further
          if (intersectionCount == 1) {
            pointToChange = intersectionPoint1;
          } else {
            // Must be 2 intersections, choose the one even further from the end
            if (math::lessThan(math::distance(intersectionPoint1, pointToChange), math::distance(intersectionPoint2, pointToChange))) {
              edgeEnd = intersectionPoint2;
            } else {
              edgeEnd = intersectionPoint1;
            }
          }
        }
      }
    }
    for (int tmpFunnelIndex=funnel_.size()-2; tmpFunnelIndex>funnel_.apex_index(); --tmpFunnelIndex) {
      const Vector &tmpFunnelPoint = funnel_.at(tmpFunnelIndex);
      Vector intersectionPoint1, intersectionPoint2;
      const int intersectionCount = math::lineSegmentIntersectsWithCircle(edgeStart, edgeEnd, tmpFunnelPoint, agentRadius_, &intersectionPoint1, &intersectionPoint2);
      if (intersectionCount > 0) {
        // Intersects with a point of another vertex, need to move it further
        if (intersectionCount == 1) {
          if (math::lessThan(math::distance(intersectionPoint1, edgeStart), math::distance(edgeEnd, edgeStart))) {
            // New point is closer to the start, need to move
            edgeEnd = intersectionPoint1;
          }
        } else {
          // Must be 2 intersections, choose the one even further from the end
          if (math::lessThan(math::distance(intersectionPoint1, edgeStart), math::distance(intersectionPoint2, edgeStart))) {
            edgeEnd = intersectionPoint1;
          } else {
            edgeEnd = intersectionPoint2;
          }
        }
      }
    }
  }

  // std::cout << "  Trimmed edge to " << DebugLogger::instance().pointToString(edgeStart) << " -> " << DebugLogger::instance().pointToString(edgeEnd) << std::endl; //DEBUGPRINTS

  if (funnel_.size() == 3) {
    // Funnel is just a triangle that ends in the goal edge. We can take a straight path to the goal segment
    if (math::distance(edgeStart, funnel_.apex_point()) < math::distance(edgeEnd, funnel_.apex_point())) {
      // Start of the edge is closer to the apex
      return edgeStart;
    } else {
      // End of the edge is closer to the apex
      return edgeEnd;
    }
  }

  // For each point in the funnel, find the closest point on the target edge, then check what the overall funnel length is
  AngleDirection funnelApexAngleDirection = AngleDirection::kCounterclockwise;
  for (int funnelIndex=0; funnelIndex<funnel_.size(); ++funnelIndex) {
    const Vector &currentFunnelPoint = funnel_.at(funnelIndex);
    if (currentFunnelPoint == funnel_.apex_point()) {
      // We've reached the apex of the funnel, there are no more points left on the left of the funnel
      funnelApexAngleDirection = funnel_.apex_type();
      // std::cout << "  Evaluating apex of funnel" << std::endl; //DEBUGPRINTS
    }
    // Test that, if this was the final apex, would it result in the shortest path
    // std::cout << "  Checking distance from a point on the funnel (" << DebugLogger::instance().pointToString(currentFunnelPoint) << ") to the target (trimmed) edge (" << DebugLogger::instance().pointToString(edgeStart) << "->" << DebugLogger::instance().pointToString(edgeEnd) << ")" << std::endl; //DEBUGPRINTS
    Vector potentialGoal;
    math::distanceBetweenEdgeAndCircleTangentIntersectionPoint(edgeStart, edgeEnd, currentFunnelPoint, agentRadius_, funnelApexAngleDirection, &potentialGoal);

    const double remainingPathLength = funnelLengthForAgentWithRadius(*this, potentialGoal);
    // std::cout << "  Remaining path length: " << remainingPathLength << std::endl; //DEBUGPRINTS
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

  if (result) {
    // std::cout << "Funnel ends with a segment from apex " << DebugLogger::instance().pointToString(funnel_.apex_point()) << " to " << DebugLogger::instance().pointToString(*result) << std::endl; //DEBUGPRINTS
    return *result;
  } else {
    throw std::runtime_error("No point found!");
  }
}

double LengthFunnel::currentPathLength() const {
  return length_;
}

// ===================================================================================
// ============================Funnel data structure below============================
// ===================================================================================

Funnel::Funnel(const Vector &initialApex, const std::size_t corridorSize) : funnel_(1 + corridorSize*2 + 2) {
  // Allocate enough space for the entire corridor to fit on either side, +1 for the apex, and +2 for the possibility of the goal on either side
  apexIndex_ = static_cast<int>(funnel_.size()/2);
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

const Vector& Funnel::at(int index) const {
  if (index < 0 || index >= size()) {
    throw std::runtime_error("Accessing funnel out of bounds");
  }
  return funnel_.at(leftIndex_+index);
}

const Vector& Funnel::reverse_at(int index) const {
  return at(size()-1-index);
}

const Vector& Funnel::front() const {
  return funnel_.at(leftIndex_);
}

void Funnel::pop_front() {
  ++leftIndex_;
}

void Funnel::push_front(const Vector &point) {
  --leftIndex_;
  funnel_.at(leftIndex_) = point;
}

const Vector& Funnel::back() const {
  return funnel_.at(rightIndex_);
}

void Funnel::pop_back() {
  --rightIndex_;
}

void Funnel::push_back(const Vector &point) {
  ++rightIndex_;
  funnel_.at(rightIndex_) = point;
}

bool Funnel::point_in_funnel(const Vector &point) const {
  const auto endIt = funnel_.begin() + leftIndex_ + size();
  return std::find(funnel_.begin()+leftIndex_, endIt, point) != endIt;
}

Apex Funnel::funnel_apex() const {
  return Apex{apex_point(), apex_type()};
}

const Vector& Funnel::apex_point() const {
  return funnel_.at(apexIndex_);
}

AngleDirection Funnel::apex_type() const {
  return apexType_;
}

int Funnel::apex_index() const {
  return apexIndex_-leftIndex_;
}

int Funnel::reverse_apex_index() const {
  return size()-1-apex_index();
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

} // namespace pathfinder
