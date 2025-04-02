#ifndef PATHFINDER_DEBUGLOGGER_H_
#define PATHFINDER_DEBUGLOGGER_H_

#include "vector.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace pathfinder {

struct Apex;
class Funnel;

class DebugLogger {
public:
  static DebugLogger& instance();
  void setPointToIndexFunction(const std::function<std::optional<uint32_t>(const Vector &point)> &func);

  void printFunnel(const Funnel &funnel) const;
  std::string pointToString(const Vector &point) const;
  void setStartPoint(const Vector &point);
  void setGoalPoint(const Vector &point);
  void resetStartPoint();
  void resetGoalPoint();

private:
  bool initialized_{false};
  std::optional<Vector> startPoint_;
  std::optional<Vector> goalPoint_;
  std::function<std::optional<uint32_t>(const Vector &point)> pointToIndexFunction_;

  DebugLogger();
  std::string apexToString(const Apex &apex) const;
};

} // namespace pathfinder

#endif // PATHFINDER_DEBUGLOGGER_H_