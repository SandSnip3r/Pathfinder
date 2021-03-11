#include "vector.h"

Vector::Vector(const double x, const double y) : x_(x), y_(y) {

}

bool Vector::operator==(const Vector &other) const {
  return (x_ == other.x_) && (y_ == other.y_);
}

bool Vector::operator!=(const Vector &other) const {
  return (x_ != other.x_) || (y_ != other.y_);
}

void Vector::setX(const double x) {
  x_ = x;
}

void Vector::setY(const double y) {
  y_ = y;
}

double Vector::x() const {
  return x_;
}

double Vector::y() const {
  return y_;
}