/*
***********************************************************************
* aabox2d_test.cc: (undirected) axes-aligned bounding boxes in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/aabox2d.h"
#include <boost/test/included/unit_test.hpp>

using namespace ASV::common::math;

BOOST_AUTO_TEST_CASE(GetAllCorners) {
  AABox2d box1(Vec2d(0, 0), 4, 2);
  auto corners1 = box1.GetAllCorners();

  BOOST_CHECK_CLOSE(corners1[0].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[0].y(), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[1].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[1].y(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[2].x(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[2].y(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[3].x(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[3].y(), -1.0, 1e-6);

  AABox2d box2(Vec2d(3, 1), Vec2d(7, 3));
  auto corners2 = box2.GetAllCorners();
  BOOST_CHECK_CLOSE(corners2[0].x(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[0].y(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[1].x(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[1].y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[2].x(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[2].y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[3].x(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(corners2[3].y(), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(CenterAndLengths) {
  AABox2d box1(Vec2d(0, 0), 10, 6);
  BOOST_CHECK_CLOSE(box1.center_x(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.center_y(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.length(), 10.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.width(), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.half_length(), 5.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.half_width(), 3.0, 1e-6);

  std::vector<Vec2d> points;
  points.push_back(Vec2d(0, 2));
  points.push_back(Vec2d(0, -6));
  points.push_back(Vec2d(3, 0));
  points.push_back(Vec2d(1, 0));

  AABox2d box2(points);
  BOOST_CHECK_CLOSE(box2.center_x(), 1.5, 1e-6);
  BOOST_CHECK_CLOSE(box2.center_y(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.length(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box2.half_length(), 1.5, 1e-6);
  BOOST_CHECK_CLOSE(box2.half_width(), 4.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(HasOverlap) {
  AABox2d box1(Vec2d(0, 0), 4, 2);
  AABox2d box2(Vec2d(3, 1), Vec2d(7, 3));
  AABox2d box3(Vec2d(0, 0), 10, 10);

  BOOST_TEST(!box1.HasOverlap(box2));
  BOOST_TEST(box1.HasOverlap(box3));
  BOOST_TEST(box2.HasOverlap(box3));
}

BOOST_AUTO_TEST_CASE(DistanceTo) {
  AABox2d box(Vec2d(0, 0), 4, 2);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(3, 0)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(-3, 0)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(0, 2)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(0, -2)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(0, 0)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(0, 1)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(1, 0)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(0, -1)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box.DistanceTo(Vec2d(-1, 0)), 0.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_TEST(box.IsPointIn(Vec2d(0, 0)));
  BOOST_TEST(box.IsPointIn(Vec2d(1, 0.5)));
  BOOST_TEST(box.IsPointIn(Vec2d(-0.5, -1)));
  BOOST_TEST(box.IsPointIn(Vec2d(2, 1)));
  BOOST_TEST(!box.IsPointIn(Vec2d(-3, 0)));
  BOOST_TEST(!box.IsPointIn(Vec2d(0, 2)));
  BOOST_TEST(!box.IsPointIn(Vec2d(-4, -2)));
}

BOOST_AUTO_TEST_CASE(IsPointOnBoundary) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_TEST(!box.IsPointOnBoundary(Vec2d(0, 0)));
  BOOST_TEST(!box.IsPointOnBoundary(Vec2d(1, 0.5)));
  BOOST_TEST(box.IsPointOnBoundary(Vec2d(-0.5, -1)));
  BOOST_TEST(box.IsPointOnBoundary(Vec2d(2, 0.5)));
  BOOST_TEST(box.IsPointOnBoundary(Vec2d(-2, 1)));
  BOOST_TEST(!box.IsPointOnBoundary(Vec2d(-3, 0)));
  BOOST_TEST(!box.IsPointOnBoundary(Vec2d(0, 2)));
  BOOST_TEST(!box.IsPointOnBoundary(Vec2d(-4, -2)));
}

BOOST_AUTO_TEST_CASE(MinMax) {
  AABox2d box({0, 0}, 4, 2);
  BOOST_CHECK_CLOSE(box.min_x(), -2, 1e-6);
  BOOST_CHECK_CLOSE(box.max_x(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box.min_y(), -1, 1e-6);
  BOOST_CHECK_CLOSE(box.max_y(), 1, 1e-6);

  AABox2d box2(Vec2d(3, 1), Vec2d(7, 3));
  BOOST_CHECK_CLOSE(box2.min_x(), 3, 1e-6);
  BOOST_CHECK_CLOSE(box2.max_x(), 7, 1e-6);
  BOOST_CHECK_CLOSE(box2.min_y(), 1, 1e-6);
  BOOST_CHECK_CLOSE(box2.max_y(), 3, 1e-6);
}

BOOST_AUTO_TEST_CASE(Shift) {
  AABox2d box({0, 0}, 4, 2);
  box.Shift(Vec2d(30, 40));
  auto corners = box.GetAllCorners();
  BOOST_CHECK_CLOSE(corners[0].x(), 32.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[0].y(), 39.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].x(), 32.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].y(), 41.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].x(), 28.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].y(), 41.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].x(), 28.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].y(), 39.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(MergeFrom) {
  AABox2d box(Vec2d(3, 1), Vec2d(7, 3));
  box.MergeFrom(AABox2d(Vec2d(5, -1), Vec2d(10, 7)));
  BOOST_CHECK_CLOSE(box.center_x(), 6.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center_y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 3.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 4.0, 1e-6);

  box.MergeFrom(Vec2d(6, 6));
  BOOST_CHECK_CLOSE(box.center_x(), 6.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center_y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 3.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 4.0, 1e-6);

  box.MergeFrom(Vec2d(-5, 20));
  BOOST_CHECK_CLOSE(box.center_x(), 2.5, 1e-6);
  BOOST_CHECK_CLOSE(box.center_y(), 9.5, 1e-6);
  BOOST_CHECK_CLOSE(box.length(), 15, 1e-6);
  BOOST_CHECK_CLOSE(box.width(), 21, 1e-6);
  BOOST_CHECK_CLOSE(box.half_length(), 7.5, 1e-6);
  BOOST_CHECK_CLOSE(box.half_width(), 10.5, 1e-6);
}