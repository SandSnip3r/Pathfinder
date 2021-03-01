#ifndef DEBUGLOGGER_H
#define DEBUGLOGGER_H

#include "funnel.h"

#include <QPointF>

#include <functional>
#include <optional>
#include <string>

class DebugLogger {
public:
  static DebugLogger& instance();
  void setPointToIndexFunction(const std::function<int(const QPointF &point)> &func);

  void printFunnel(const Funnel &funnel) const;
  std::string pointToString(const QPointF &point) const;
  void setStartPoint(const QPointF &point);
  void setGoalPoint(const QPointF &point);
  void resetStartPoint();
  void resetGoalPoint();

private:
  bool initialized_{false};
  std::optional<QPointF> startPoint_;
  std::optional<QPointF> goalPoint_;
  std::function<int(const QPointF &point)> pointToIndexFunction_;

  DebugLogger();
  std::string DebugLogger::apexToString(const Apex &apex) const;
};

#endif // DEBUGLOGGER_H
