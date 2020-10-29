/*
***********************************************************************
* linesegment2d_test.cc: test for Line segment in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/linesegment2d.h"
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(Accessors) {
  ASV::common::math::LineSegment2d ls(ASV::common::math::Vec2d(1, 2),
                                      ASV::common::math::Vec2d(5, 4));
  BOOST_CHECK_CLOSE(ls.length(), std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.length_sqr(), 20.0, 1e-6);
  BOOST_CHECK_CLOSE(ls.center().x(), 3, 1e-6);
  BOOST_CHECK_CLOSE(ls.center().y(), 3, 1e-6);
  BOOST_CHECK_CLOSE(ls.heading(), std::atan2(2, 4), 1e-6);
  BOOST_CHECK_CLOSE(ls.cos_heading(), 4.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.sin_heading(), 2.0 / std::sqrt(20.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(Distance) {
  ASV::common::math::LineSegment2d ls(ASV::common::math::Vec2d(1, 2),
                                      ASV::common::math::Vec2d(5, 4));
  auto [distance1, nearest_pt1] = ls.DistanceTo(ASV::common::math::Vec2d(0, 0));

  BOOST_CHECK_CLOSE(distance1, std::sqrt(5.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt1.x(), 1, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt1.y(), 2, 1e-6);

  auto [distance2, nearest_pt2] =
      ls.DistanceTo(ASV::common::math::Vec2d(10, 10));
  BOOST_CHECK_CLOSE(distance2, std::sqrt(61.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt2.x(), 5, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt2.y(), 4, 1e-6);

  auto [distance3, nearest_pt3] = ls.DistanceTo(ASV::common::math::Vec2d(3, 3));
  BOOST_CHECK_CLOSE(distance3, 0, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt3.x(), 3, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt3.y(), 3, 1e-6);

  auto [distance4, nearest_pt4] = ls.DistanceTo(ASV::common::math::Vec2d(4, 4));
  BOOST_CHECK_CLOSE(distance4, 2.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt4.x(), 4.2, 1e-6);
  BOOST_CHECK_CLOSE(nearest_pt4.y(), 3.6, 1e-6);
}

BOOST_AUTO_TEST_CASE(PerpendicularFoot) {
  ASV::common::math::LineSegment2d ls(ASV::common::math::Vec2d(1, 2),
                                      ASV::common::math::Vec2d(5, 4));

  auto [distance1, foot_pt1] =
      ls.GetPerpendicularFoot(ASV::common::math::Vec2d(0, 0));
  BOOST_CHECK_CLOSE(distance1, 0.6 * std::sqrt(5.0), 1e-6);
  BOOST_CHECK_CLOSE(foot_pt1.x(), -0.6, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt1.y(), 1.2, 1e-6);

  auto [distance2, foot_pt2] =
      ls.GetPerpendicularFoot(ASV::common::math::Vec2d(3, 3));
  BOOST_CHECK_CLOSE(distance2, 0, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt2.x(), 3, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt2.y(), 3, 1e-6);

  auto [distance3, foot_pt3] = ls.DistanceTo(ASV::common::math::Vec2d(4, 4));
  BOOST_CHECK_CLOSE(distance3, 2.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(foot_pt3.x(), 4.2, 1e-6);
  BOOST_CHECK_CLOSE(foot_pt3.y(), 3.6, 1e-6);
}

BOOST_AUTO_TEST_CASE(ProjectOntoUnit) {
  ASV::common::math::LineSegment2d ls(ASV::common::math::Vec2d(1, 2),
                                      ASV::common::math::Vec2d(5, 4));
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit(ASV::common::math::Vec2d(1, 2)), 0.0,
                    1e-6);
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit(ASV::common::math::Vec2d(5, 4)),
                    std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit(ASV::common::math::Vec2d(-2, -3)),
                    -22.0 / std::sqrt(20.0), 1e-6);
  BOOST_CHECK_CLOSE(ls.ProjectOntoUnit(ASV::common::math::Vec2d(6, 7)),
                    30.0 / std::sqrt(20.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(GetIntersect) {
  using namespace ASV::common::math;

  LineSegment2d ls(Vec2d(1, 2), Vec2d(5, 4));

  auto [isIntersect1, point1] =
      ls.GetIntersect(LineSegment2d(Vec2d(1, 3), Vec2d(5, 5)));
  BOOST_TEST(!isIntersect1);
  BOOST_CHECK_CLOSE(point1.x(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(point1.y(), 0.0, 1e-6);

  auto [isIntersect2, point2] =
      ls.GetIntersect(LineSegment2d(Vec2d(2, 2), Vec2d(6, 4)));
  BOOST_TEST(!isIntersect2);
  BOOST_CHECK_CLOSE(point2.x(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(point2.y(), 0.0, 1e-6);

  auto [isIntersect3, point3] =
      ls.GetIntersect(LineSegment2d(Vec2d(1, 2), Vec2d(-3, 0)));
  BOOST_TEST(isIntersect3);
  BOOST_CHECK_CLOSE(point3.x(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(point3.y(), 2.0, 1e-6);

  auto [isIntersect4, point4] =
      ls.GetIntersect(LineSegment2d(Vec2d(5, 4), Vec2d(9, 6)));
  BOOST_TEST(isIntersect4);
  BOOST_CHECK_CLOSE(point4.x(), 5.0, 1e-6);
  BOOST_CHECK_CLOSE(point4.y(), 4.0, 1e-6);

  auto [isIntersect5, point5] =
      ls.GetIntersect(LineSegment2d(Vec2d(3, 0), Vec2d(3, 10)));
  BOOST_TEST(isIntersect5);
  BOOST_CHECK_CLOSE(point5.x(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point5.y(), 3.0, 1e-6);

  auto [isIntersect6, point6] =
      ls.GetIntersect(LineSegment2d(Vec2d(3, 10), Vec2d(3, 0)));
  BOOST_TEST(isIntersect6);
  BOOST_CHECK_CLOSE(point6.x(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point6.y(), 3.0, 1e-6);

  auto [isIntersect7, point7] =
      ls.GetIntersect(LineSegment2d(Vec2d(3, 3), Vec2d(3, 3)));
  BOOST_TEST(isIntersect7);
  BOOST_CHECK_CLOSE(point7.x(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(point7.y(), 3.0, 1e-6);

  auto [isIntersect8, point8] =
      ls.GetIntersect(LineSegment2d(Vec2d(3, 5), Vec2d(3, 10)));
  BOOST_TEST(!isIntersect8);

  auto [isIntersect9, point9] =
      ls.GetIntersect(LineSegment2d(Vec2d(3, 2), Vec2d(3, 0)));
  BOOST_TEST(!isIntersect9);

  auto [isIntersect10, point10] =
      ls.GetIntersect(LineSegment2d(Vec2d(4, 4), Vec2d(4, 4)));
  BOOST_TEST(!isIntersect10);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  using namespace ASV::common::math;
  LineSegment2d ls(Vec2d(1, 2), Vec2d(5, 4));

  BOOST_TEST(ls.IsPointIn(Vec2d(1, 2)));
  BOOST_TEST(ls.IsPointIn(Vec2d(5, 4)));
  BOOST_TEST(ls.IsPointIn(Vec2d(3, 3)));
  BOOST_TEST(!ls.IsPointIn(Vec2d(-1, 1)));
  BOOST_TEST(!ls.IsPointIn(Vec2d(7, 5)));
  BOOST_TEST(!ls.IsPointIn(Vec2d(0, 0)));
  BOOST_TEST(!ls.IsPointIn(Vec2d(6, 6)));
}