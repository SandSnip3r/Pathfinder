#include "math_helpers.h"
#include "vector.h"

namespace pathfinder {

Vector::Vector(const double x, const double y) : x_(x), y_(y) {

}

bool Vector::operator==(const Vector &other) const {
  return math::equal(x_, other.x_) && math::equal(y_, other.y_);
}

bool Vector::operator!=(const Vector &other) const {
  return !math::equal(x_, other.x_) || !math::equal(y_, other.y_);
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

Vector operator*(const Vector &lhs, double scalar) {
  return {lhs.x_ * scalar, lhs.y_ * scalar};
}

Vector operator*(double scalar, const Vector &rhs) {
  return {rhs.x_ * scalar, rhs.y_ * scalar};
}

Vector operator/(const Vector &lhs, double scalar) {
  return {lhs.x_ / scalar, lhs.y_ / scalar};
}

Vector operator/(double scalar, const Vector &rhs) {
  return {rhs.x_ / scalar, rhs.y_ / scalar};
}

Vector operator-(const Vector &lhs, const Vector &rhs) {
  return {lhs.x_-rhs.x_, lhs.y_-rhs.y_};
}

Vector operator+(const Vector &lhs, const Vector &rhs) {
  return {lhs.x_+rhs.x_, lhs.y_+rhs.y_};
}

std::ostream& operator<<(std::ostream &stream, const pathfinder::Vector &vector) {
  stream << '(' << vector.x() << ',' << vector.y() << ')';
  return stream;
}

} // namespace pathfinder
