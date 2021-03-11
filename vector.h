#ifndef VECTOR_H_
#define VECTOR_H_

class Vector {
private:
  double x_, y_;
public:
  Vector() = default;
  Vector(const double x, const double y);

  bool operator==(const Vector &other) const;
  bool operator!=(const Vector &other) const;
  
  void setX(const double x);
  void setY(const double y);
  double x() const;
  double y() const;
};

#endif // VECTOR_H_