#pragma once
#include <cmath>

namespace teledrive {

struct Vec2 {
  double x{0.0};
  double y{0.0};

  constexpr Vec2() = default;
  constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

  constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }

  constexpr double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  double norm() const { return std::sqrt(dot(*this)); }
};

}  // namespace teledrive
