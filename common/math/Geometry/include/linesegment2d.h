/*
***********************************************************************
* linesegment2d.h: Line segment in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _LINESEGMENT2D_H_
#define _LINESEGMENT2D_H_

#include "common/math/miscellaneous/include/math_utils.h"

namespace ASV::common::math {

class LineSegment2d {
 public:
  LineSegment2d()
      : start_(Vec2d(0, 0)),
        end_(Vec2d(0, 0)),
        unit_direction_(Vec2d(1, 0)),
        heading_(0),
        length_(0) {}

  LineSegment2d(const Vec2d &start, const Vec2d &end)
      : start_(start),
        end_(end),
        unit_direction_(Vec2d(1, 0)),
        heading_(0),
        length_(0) {
    const double dx = end_.x() - start_.x();
    const double dy = end_.y() - start_.y();
    length_ = std::hypot(dx, dy);
    unit_direction_ =
        (length_ <= kMathEpsilon ? Vec2d(0, 0)
                                 : Vec2d(dx / length_, dy / length_));
    heading_ = unit_direction_.Angle();
  }
  virtual ~LineSegment2d() = default;

  Vec2d start() const noexcept { return start_; }
  Vec2d end() const noexcept { return end_; }
  Vec2d unit_direction() const noexcept { return unit_direction_; }

  // Get the center of the line segment.
  Vec2d center() const { return (start_ + end_) * 0.5; }  // center

  /** @brief Get a new line-segment with the same start point, but rotated
   * counterclock-wise by the given amount.
   * @return The rotated line-segment's end-point.
   */
  Vec2d rotate(const double angle) const {
    Vec2d diff_vec = end_ - start_;
    diff_vec.SelfRotate(angle);
    return start_ + diff_vec;
  }  // rotate

  double heading() const noexcept { return heading_; }

  // Get the cosine of the heading.
  double cos_heading() const noexcept {
    return unit_direction_.x();
  }  // cos_heading

  // @brief Get the sine of the heading.
  double sin_heading() const noexcept {
    return unit_direction_.y();
  }  // sin_heading

  double length() const noexcept { return length_; }
  // Get the square of length of the line segment.
  double length_sqr() const noexcept { return length_ * length_; }

  // find the nearst points on the line segment
  Vec2d find_nearest_point(const Vec2d &point) const {
    if (length_ <= kMathEpsilon) {
      return start_;
    }

    double proj = ProjectOntoUnit(point);

    if (proj <= 0.0) {
      return start_;
    } else if (proj >= length_) {
      return end_;
    } else {
      return start_ + unit_direction_ * proj;
    }
  }  // find_nearest_point

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  std::pair<double, Vec2d> GetPerpendicularFoot(const Vec2d &point) const {
    if (length_ <= kMathEpsilon) {
      return DistanceTo(start_);
    }

    double proj = ProjectOntoUnit(point);

    return {std::abs(ProductOntoUnit(point)), start_ + unit_direction_ * proj};
  }  // GetPerpendicularFoot

  // Compute the shortest distance from a point on the line segment to a point
  // in 2-D.
  std::pair<double, Vec2d> DistanceTo(const Vec2d &point) const {
    Vec2d nearstpoint = find_nearest_point(point);

    return {(nearstpoint - point).Length(), nearstpoint};
  }  // DistanceTo

  //  Check if a point is within the line segment.
  bool IsPointIn(const Vec2d &point) const {
    if (length_ <= kMathEpsilon) {
      return (start_ == point);
    }

    double prod = CrossProd(start_ - point, end_ - point);
    if (std::abs(prod) > kMathEpsilon) {
      return false;
    }
    return IsWithin(point.x(), start_.x(), end_.x()) &&
           IsWithin(point.y(), start_.y(), end_.y());
  }

  // Check if the line segment has an intersect with another 2d line segment
  bool HasIntersect(const LineSegment2d &other_segment) const {
    auto [IsIntersect, point] = GetIntersect(other_segment);
    (void)point;  // unused
    return IsIntersect;
  }  // HasIntersect

  // Compute the intersect with another line segment in 2-D if any.
  /* @param other_segment The line segment to compute the intersect.
   * @return Whether the line segment has an intersect
   *         with the input other_segment, and point the computed intersect
   *          between the line segment and the input other_segment.
   */
  std::pair<bool, Vec2d> GetIntersect(
      const LineSegment2d &other_segment) const {
    if (IsPointIn(other_segment.start())) {
      return {true, other_segment.start()};
    }
    if (IsPointIn(other_segment.end())) {
      return {true, other_segment.end()};
    }
    if (other_segment.IsPointIn(start_)) {
      return {true, start_};
    }
    if (other_segment.IsPointIn(end_)) {
      return {true, end_};
    }

    if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
      return {false, Vec2d(0, 0)};
    }
    const double cc1 = CrossProd(end_ - start_, other_segment.start() - start_);
    const double cc2 = CrossProd(end_ - start_, other_segment.end() - start_);
    if (cc1 * cc2 >= -kMathEpsilon) {
      return {false, Vec2d(0, 0)};
    }
    const double cc3 = CrossProd(other_segment.end() - other_segment.start(),
                                 start_ - other_segment.start());
    const double cc4 = CrossProd(other_segment.end() - other_segment.start(),
                                 end_ - other_segment.start());
    if (cc3 * cc4 >= -kMathEpsilon) {
      return {false, Vec2d(0, 0)};
    }

    const double ratio = cc4 / (cc4 - cc3);
    return {true, Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                        start_.y() * ratio + end_.y() * (1.0 - ratio))};

  }  // GetIntersect

  // Compute the projection of a vector onto the line segment,
  // which is from the start point of the line segment to the input point,
  // onto the unit direction
  double ProjectOntoUnit(const Vec2d &point) const {
    auto vectorps = point - start_;
    return InnerProd(vectorps.x(), vectorps.y(), unit_direction_.x(),
                     unit_direction_.y());
  }

  // Compute the cross product of a vector onto the line segment,
  // which is from the start point of the line segment to the input point.
  double ProductOntoUnit(const Vec2d &point) const {
    auto vectorps = point - start_;
    return CrossProd(unit_direction_.x(), unit_direction_.y(), vectorps.x(),
                     vectorps.y());
  }

 private:
  Vec2d start_;
  Vec2d end_;
  // the unit direction from the start point to the end point.
  Vec2d unit_direction_;
  double heading_;
  double length_;
};  // end class LineSegment2d

}  // namespace ASV::common::math

#endif /* _LINESEGMENT2D_H_ */