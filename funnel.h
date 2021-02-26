#ifndef FUNNEL_H
#define FUNNEL_H

#include "math_helpers.h"

#include <QPointF>

#include <vector>

struct Apex {
  QPointF apexPoint;
  AngleDirection apexType;
};

class Funnel {
public:
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

#endif // FUNNEL_H
