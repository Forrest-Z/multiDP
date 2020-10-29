/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _VEC2D_H_
#define _VEC2D_H_

#include <cmath>

namespace ASV::common::math {

constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors. (double)
 */
class Vec2d {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2d CreateUnitVec2d(const double angle) {
    return Vec2d(std::cos(angle), std::sin(angle));
  }

  //! Getter for x component
  double x() const noexcept { return x_; }

  //! Getter for y component
  double y() const noexcept { return y_; }

  //! Setter for x component
  void set_x(const double x) noexcept { x_ = x; }

  //! Setter for y component
  void set_y(const double y) noexcept { y_ = y; }

  //! Gets the length of the vector
  double Length() const { return std::hypot(x_, y_); }

  //! Gets the squared length of the vector
  double LengthSquare() const { return x_ * x_ + y_ * y_; }

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const { return std::atan2(y_, x_); }

  // check the length of vector, if too small return true
  bool IsSmall() const { return (Length() <= kMathEpsilon); }

  //! Returns the unit vector that is co-linear with this vector
  void Normalize() {
    const double l = Length();
    if (l > kMathEpsilon) {
      x_ /= l;
      y_ /= l;
    }
  }  // Normalize

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2d &other) const {
    return std::hypot(x_ - other.x(), y_ - other.y());
  }

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2d &other) const {
    const double dx = x_ - other.x_;
    const double dy = y_ - other.y_;
    return dx * dx + dy * dy;
  }

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Vec2d &other) const {
    return x_ * other.y() - y_ * other.x();
  }

  //! Returns the inner product between these two Vec2d.
  double InnerProd(const Vec2d &other) const {
    return x_ * other.x() + y_ * other.y();
  }

  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const {
    return Vec2d(x_ * std::cos(angle) - y_ * std::sin(angle),
                 x_ * std::sin(angle) + y_ * std::cos(angle));
  }

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle) {
    double tmp_x = x_;
    x_ = x_ * std::cos(angle) - y_ * std::sin(angle);
    y_ = tmp_x * std::sin(angle) + y_ * std::cos(angle);
  }

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d &other) const {
    return Vec2d(x_ + other.x(), y_ + other.y());
  }

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d &other) const {
    return Vec2d(x_ - other.x(), y_ - other.y());
  }

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(const double scalar) const {
    return Vec2d(x_ * scalar, y_ * scalar);
  }

  //! Sums another Vec2d to the current one
  Vec2d &operator+=(const Vec2d &other) {
    x_ += other.x();
    y_ += other.y();
    return *this;
  }

  //! Subtracts another Vec2d to the current one
  Vec2d &operator-=(const Vec2d &other) {
    x_ -= other.x();
    y_ -= other.y();
    return *this;
  }

  //! Multiplies this Vec2d by a scalar
  Vec2d &operator*=(const double ratio) {
    x_ *= ratio;
    y_ *= ratio;
    return *this;
  }

  //! Compares two Vec2d
  bool operator==(const Vec2d &other) const {
    return (std::abs(x_ - other.x()) < kMathEpsilon &&
            std::abs(y_ - other.y()) < kMathEpsilon);
  }

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

}  // namespace ASV::common::math

#endif /* _VEC2D_H_ */