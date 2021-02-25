#include "funnel.h"

#include <algorithm>
#include <stdexcept>

Funnel::Funnel(const QPointF &initialApex, const int corridorSize) : funnel(1 + corridorSize*2 + 2) {
  apexIndex = funnel.size()/2;
  funnel.at(apexIndex) = initialApex;
  leftIndex = apexIndex;
  rightIndex = apexIndex;
}

int Funnel::size() const {
  return (rightIndex-leftIndex) + 1;
}

bool Funnel::empty() const {
  if (leftIndex > rightIndex) {
    throw std::runtime_error("Funnel is empty. Shouldnt be possible");
  }
  return false;
}

const QPointF& Funnel::at(int index) const {
  if (index >= size()) {
    throw std::runtime_error("Accessing funnel out of bounds");
  }
  return funnel.at(leftIndex+index);
}

const QPointF& Funnel::front() const {
  return funnel.at(leftIndex);
}

void Funnel::pop_front() {
  ++leftIndex;
}

void Funnel::push_front(const QPointF &point) {
  --leftIndex;
  funnel.at(leftIndex) = point;
}

const QPointF& Funnel::back() const {
  return funnel.at(rightIndex);
}

void Funnel::pop_back() {
  --rightIndex;
}

void Funnel::push_back(const QPointF &point) {
  ++rightIndex;
  funnel.at(rightIndex) = point;
}

bool Funnel::point_in_funnel(const QPointF &point) const {
  return std::find(funnel.begin()+leftIndex, (funnel.begin()+leftIndex+size()), point) != (funnel.begin()+leftIndex+size());
}