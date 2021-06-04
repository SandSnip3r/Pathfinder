#include "debuglogger.h"
#include "funnel.h"
#include "math_helpers.h"

#include <iostream>
#include <stdexcept>

namespace pathfinder {

DebugLogger& DebugLogger::instance() {
  static DebugLogger thisInstance = DebugLogger();
  return thisInstance;
}

DebugLogger::DebugLogger() {

}

void DebugLogger::setPointToIndexFunction(const std::function<std::optional<uint32_t>(const Vector &point)> &func) {
  pointToIndexFunction_ = func;
  initialized_ = true;
}

void DebugLogger::printFunnel(const Funnel &funnel) const {
  if (!initialized_) {
    throw std::runtime_error("DebugLogger unitialized");
  }
  std::cout << "[";
  for (int i=0; i<funnel.size(); ++i) {
    if (i == funnel.apex_index()) {
      std::cout << apexToString(funnel.funnel_apex());
    } else {
      std::cout << pointToString(funnel.at(i));
    }
    std::cout << ',';
  }
  std::cout << "]" << std::endl;
}

std::string DebugLogger::pointToString(const Vector &point) const {
  if (!initialized_) {
    throw std::runtime_error("DebugLogger unitialized");
  }
  if (startPoint_ && point == *startPoint_) {
    return "[FUNNEL START]";
  } else if (goalPoint_ && point == *goalPoint_) {
    return "[FUNNEL GOAL]";
  } else {
    const auto pointIndex = pointToIndexFunction_(point);
    if (pointIndex) {
      if constexpr (std::is_same_v<decltype(pointIndex)::value_type, uint32_t>) {
        return "("+std::to_string((*pointIndex)>>16)+","+std::to_string((*pointIndex)&0xFFFF)+")";
      } else {
        return std::to_string(*pointIndex);
      }
    } else {
      std::string result = "(";
      result += std::to_string(point.x());
      result += ",";
      result += std::to_string(point.y());
      result += ")";
      return result;
    }
  }
}

void DebugLogger::setStartPoint(const Vector &point) {
  startPoint_ = point;
}

void DebugLogger::setGoalPoint(const Vector &point) {
  goalPoint_ = point;
}

void DebugLogger::resetStartPoint() {
  startPoint_.reset();
}

void DebugLogger::resetGoalPoint() {
  goalPoint_.reset();
}

std::string DebugLogger::apexToString(const Apex &apex) const {
  std::string result = "(" + pointToString(apex.apexPoint);
  if (apex.apexType == AngleDirection::kClockwise) {
    result += ",cw";
  } else if (apex.apexType == AngleDirection::kCounterclockwise) {
    result += ",ccw";
  }
  result += ")";
  return result;
}

} // namespace pathfinder
