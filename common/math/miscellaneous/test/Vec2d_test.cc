/*
***********************************************************************
* testmathutils.h: Test math utility function
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/Vec2d.h"
#include <boost/test/included/unit_test.hpp>
#include <iostream>

using namespace ASV::common::math;

BOOST_AUTO_TEST_CASE(NomralCases) {
  Vec2d pt(2, 3);

  BOOST_CHECK_CLOSE(pt.Length(), std::sqrt(13.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.LengthSquare(), 13.0, 1e-7);
  BOOST_CHECK_CLOSE(pt.DistanceTo({0, 0}), std::sqrt(13.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.DistanceSquareTo({0, 0}), 13.0, 1e-7);
  BOOST_CHECK_CLOSE(pt.DistanceTo({0, 2}), std::sqrt(5.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.DistanceSquareTo({0, 2}), 5.0, 1e-7);
  BOOST_CHECK_CLOSE(pt.Angle(), std::atan2(3, 2), 1e-7);
  BOOST_CHECK_CLOSE(pt.CrossProd({4, 5}), -2, 1e-7);
  BOOST_CHECK_CLOSE(pt.InnerProd({4, 5}), 23, 1e-7);

  pt.set_x(4);
  pt.set_y(5);
  BOOST_CHECK_CLOSE(pt.Length(), std::sqrt(41.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.LengthSquare(), 41.0, 1e-7);

  pt.Normalize();
  BOOST_CHECK_CLOSE(pt.x(), 4.0 / std::sqrt(41.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.y(), 5.0 / std::sqrt(41.0), 1e-7);
  BOOST_CHECK_CLOSE(pt.Length(), 1.0, 1e-7);

  const Vec2d d = Vec2d(0.5, 1.5) + Vec2d(2.5, 3.5);
  BOOST_CHECK_CLOSE(d.x(), 3.0, 1e-7);
  BOOST_CHECK_CLOSE(d.y(), 5.0, 1e-7);

  const Vec2d e = Vec2d(0.5, 1.5) - Vec2d(2.5, 3.5);
  BOOST_CHECK_CLOSE(e.x(), -2.0, 1e-7);
  BOOST_CHECK_CLOSE(e.y(), -2.0, 1e-7);

  const Vec2d f = d * 0.5;
  BOOST_CHECK_CLOSE(f.x(), 1.5, 1e-7);
  BOOST_CHECK_CLOSE(f.y(), 2.5, 1e-7);
  const Vec2d g = e * (-3.0);
  BOOST_CHECK_CLOSE(g.x(), 6.0, 1e-7);
  BOOST_CHECK_CLOSE(g.y(), 6.0, 1e-7);

  const Vec2d unit_pt = Vec2d::CreateUnitVec2d(M_PI / 4);
  BOOST_CHECK_CLOSE(unit_pt.x(), std::sqrt(2.0) / 2.0, 1e-7);
  BOOST_CHECK_CLOSE(unit_pt.y(), std::sqrt(2.0) / 2.0, 1e-7);
  BOOST_CHECK_CLOSE(unit_pt.Angle(), M_PI / 4, 1e-7);
}

BOOST_AUTO_TEST_CASE(rotate) {
  Vec2d pt(4, 0);
  auto p1 = pt.rotate(M_PI / 2.0);
  BOOST_CHECK_SMALL(p1.x() - 0.0, 1e-7);
  BOOST_CHECK_SMALL(p1.y() - 4.0, 1e-7);
  auto p2 = pt.rotate(M_PI);
  BOOST_CHECK_SMALL(p2.x() + 4.0, 1e-7);
  BOOST_CHECK_SMALL(p2.y() - 0.0, 1e-7);
  auto p3 = pt.rotate(-M_PI / 2.0);
  BOOST_CHECK_SMALL(p3.x() - 0.0, 1e-7);
  BOOST_CHECK_SMALL(p3.y() + 4.0, 1e-7);
  auto p4 = pt.rotate(-M_PI);
  BOOST_CHECK_SMALL(p4.x() + 4.0, 1e-7);
  BOOST_CHECK_SMALL(p4.y() - 0.0, 1e-7);
}

BOOST_AUTO_TEST_CASE(selfrotate) {
  Vec2d p1(4, 0);
  p1.SelfRotate(M_PI / 2.0);
  BOOST_CHECK_SMALL(p1.x() - 0.0, 1e-7);
  BOOST_CHECK_SMALL(p1.y() - 4.0, 1e-7);
  Vec2d p2(4, 0);
  p2.SelfRotate(M_PI);
  BOOST_CHECK_SMALL(p2.x() + 4.0, 1e-7);
  BOOST_CHECK_SMALL(p2.y() - 0.0, 1e-7);
  Vec2d p3(4, 0);
  p3.SelfRotate(-M_PI / 2.0);
  BOOST_CHECK_SMALL(p3.x() - 0.0, 1e-7);
  BOOST_CHECK_SMALL(p3.y() + 4.0, 1e-7);
  Vec2d p4(4, 0);
  p4.SelfRotate(-M_PI);
  BOOST_CHECK_SMALL(p4.x() + 4.0, 1e-7);
  BOOST_CHECK_SMALL(p4.y() - 0.0, 1e-7);
}

BOOST_AUTO_TEST_CASE(Smaller) {
  Vec2d p1(0, 0);
  Vec2d p2(1, 0);

  BOOST_TEST(p1.IsSmall() == true);
  BOOST_TEST(p2.IsSmall() == false);
}