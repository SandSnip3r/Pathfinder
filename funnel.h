#ifndef FUNNEL_H
#define FUNNEL_H

#include <QPointF>

#include <vector>

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
  // const QPointF& apex() const;
private:
  std::vector<QPointF> funnel;
  int apexIndex;
  int leftIndex, rightIndex;
};

#endif // FUNNEL_H
