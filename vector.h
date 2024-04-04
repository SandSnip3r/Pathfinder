#ifndef VECTOR_H_
#define VECTOR_H_

#include <ostream>

namespace pathfinder {

class Vector {
private:
  double x_{0.0}, y_{0.0};
public:
  Vector() = default;
  Vector(const double x, const double y);

  bool operator==(const Vector &other) const;
  bool operator!=(const Vector &other) const;
  
  void setX(const double x);
  void setY(const double y);
  double x() const;
  double y() const;

  friend Vector operator*(const Vector &lhs, double scalar);
  friend Vector operator*(double scalar, const Vector &rhs);
  friend Vector operator/(const Vector &lhs, double scalar);
  friend Vector operator/(double scalar, const Vector &rhs);
  friend Vector operator-(const Vector &lhs, const Vector &rhs);
  friend Vector operator+(const Vector &lhs, const Vector &rhs);
};

Vector operator*(const Vector &lhs, double scalar);
Vector operator*(double scalar, const Vector &rhs);
Vector operator/(const Vector &lhs, double scalar);
Vector operator/(double scalar, const Vector &rhs);
Vector operator-(const Vector &lhs, const Vector &rhs);
Vector operator+(const Vector &lhs, const Vector &rhs);
std::ostream& operator<<(std::ostream &stream, const pathfinder::Vector &vector);

} // namespace pathfinder

#endif // VECTOR_H_