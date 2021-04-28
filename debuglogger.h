#ifndef DEBUGLOGGER_H
#define DEBUGLOGGER_H

#include "funnel.h"
#include "vector.h"

#include <functional>
#include <optional>
#include <string>

namespace pathfinder {

class DebugLogger {
public:
  static DebugLogger& instance();
  void setPointToIndexFunction(const std::function<int(const Vector &point)> &func);

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
  std::function<int(const Vector &point)> pointToIndexFunction_;

  DebugLogger();
  std::string apexToString(const Apex &apex) const;
};

} // namespace pathfinder

#endif // DEBUGLOGGER_H
