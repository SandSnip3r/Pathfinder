#ifndef FUNNEL_H
#define FUNNEL_H

#include "math_helpers.h"

#include <QPointF>

#include <memory>
#include <optional>
#include <vector>

struct Apex {
  QPointF apexPoint;
  AngleDirection apexType;
};

class Funnel {
public:
  Funnel() = default; // TODO: Kind of in a weird state here
  Funnel(const QPointF &initialApex, const int corridorSize);
  int size() const;
  bool empty() const;
  const QPointF& at(int index) const;
  const QPointF& front() const;
  void pop_front();
  void push_front(const QPointF &point);
  const QPointF& back() const;
  void pop_back();
  void push_back(const QPointF &point);
  bool point_in_funnel(const QPointF &point) const;
  void push_front_apex(const Apex &apex);
  void push_back_apex(const Apex &apex);
  Apex funnel_apex() const;
  const QPointF& apex_point() const;
  AngleDirection apex_type() const;
  int apex_index() const;
  void set_apex_index(const int index);
  void set_apex_type(const AngleDirection type);
private:
  std::vector<QPointF> funnel_;
  AngleDirection apexType_{AngleDirection::kPoint};
  int apexIndex_;
  int leftIndex_, rightIndex_;
};

struct PathSegment {
  virtual std::unique_ptr<PathSegment> clone() const = 0;
  virtual ~PathSegment();
};

struct StraightPathSegment : public PathSegment {
  StraightPathSegment(const QPointF &start, const QPointF &end) : startPoint(start), endPoint(end) {}
  StraightPathSegment(const StraightPathSegment &other) : startPoint(other.startPoint), endPoint(other.endPoint) {}
  virtual std::unique_ptr<PathSegment> clone() const override { return std::unique_ptr<PathSegment>(new StraightPathSegment(*this)); }
  QPointF startPoint, endPoint;
};

struct ArcPathSegment : public PathSegment {
  ArcPathSegment(const QPointF &center, const double radius, const AngleDirection direction) : circleCenter(center), circleRadius(radius), angleDirection(direction) {}
  ArcPathSegment(const ArcPathSegment &other) : circleCenter(other.circleCenter), circleRadius(other.circleRadius), angleDirection(other.angleDirection), startAngle(other.startAngle), endAngle(other.endAngle) {}
  virtual std::unique_ptr<PathSegment> clone() const override { return std::unique_ptr<PathSegment>(new ArcPathSegment(*this)); }
  QPointF circleCenter;
  double circleRadius;
  AngleDirection angleDirection;
  double startAngle, endAngle;
};

class BaseFunnel {
public:
  BaseFunnel(const double agentRadius);
  void funnel(const std::vector<std::pair<QPointF,QPointF>> &corridor, const QPointF &startPoint, std::optional<QPointF> &goalPoint);
protected:
  const double agentRadius_{0.0};
  Funnel funnel_;
  void addLeft(const QPointF &point);
  void addRight(const QPointF &point, const bool isGoal=false);
  // void addGoal(const QPointF &point); // TODO
  void finishFunnel();
private:
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) = 0;
  virtual double currentPathLength() const = 0;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const = 0;
};

class PathFunnel : public BaseFunnel {
public:
  using BaseFunnel::BaseFunnel;
  using PathType = std::vector<std::unique_ptr<PathSegment>>;
  PathType getPath() const;
private:
  PathType path_;
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const override;
};

class LengthFunnel : public BaseFunnel {
public:
  using BaseFunnel::BaseFunnel;
  double getLength() const;
private:
  double length_{0.0};
  std::optional<double> previousAngle_;
  virtual void addSegment(const Apex &previousApex, const std::pair<QPointF,QPointF> &edge, const Apex &newApex) override;
  virtual double currentPathLength() const override;
  virtual QPointF findBestGoalForFunnel(const std::pair<QPointF,QPointF> &lastEdgeOfCorridor) const override;
  double funnelLengthForAgentWithRadius(LengthFunnel funnelCopy, const QPointF &goalPoint) const; // TODO: Think
};

#endif // FUNNEL_H
