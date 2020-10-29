/*
***********************************************************************
* box2d_test.cc: (undirected) axes-aligned bounding boxes in 2-D.
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/box2d.h"
#include <boost/test/included/unit_test.hpp>

using namespace ASV::common::math;

bool CheckBoxOverlapSlow(const Box2d &box1, const Box2d &box2,
                         bool *const ambiguous) {
  double headings[4] = {box1.heading(), box1.heading() + M_PI / 2,
                        box2.heading(), box2.heading() + M_PI / 2};
  *ambiguous = false;
  for (int k = 0; k < 4; ++k) {
    const double heading = headings[k];
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    auto c1 = box1.GetAllCorners();
    auto c2 = box2.GetAllCorners();
    double s1 = std::numeric_limits<double>::infinity();
    double t1 = -std::numeric_limits<double>::infinity();
    double s2 = std::numeric_limits<double>::infinity();
    double t2 = -std::numeric_limits<double>::infinity();

    for (const auto &p : c1) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s1) {
        s1 = proj;
      }
      if (proj > t1) {
        t1 = proj;
      }
    }

    for (const auto &p : c2) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s2) {
        s2 = proj;
      }
      if (proj > t2) {
        t2 = proj;
      }
    }

    if (std::abs(s1 - t2) <= 1e-5 || std::abs(s2 - t1) <= 1e-5) {
      *ambiguous = true;
    }
    if (s1 > t2 || s2 > t1) {
      return false;
    }
  }
  return true;
}

Box2d box1(Vec2d(0, 0), 0, 4, 2);
Box2d box2(Vec2d(5, 2), 0, 4, 2);
Box2d box3(LineSegment2d(Vec2d(2, 3), Vec2d(6, 3)), 2);
Box2d box4(Vec2d(7, 8), M_PI / 4.0, 5.0, 3.0);
Box2d box5(Vec2d(-2, -3), -M_PI, 0.0, 0.0);
Box2d box6(LineSegment2d(Vec2d(2, 3), Vec2d(6, 3)), 0.0);
Box2d box7(AABox2d(Vec2d(4, 5), 0, 0));

BOOST_AUTO_TEST_CASE(GetAllCorners) {
  auto corners1 = box1.GetAllCorners();

  BOOST_CHECK_CLOSE(corners1[0].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[0].y(), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[1].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[1].y(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[2].x(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[2].y(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[3].x(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners1[3].y(), -1.0, 1e-6);

  auto corners3 = box3.GetAllCorners();

  BOOST_CHECK_CLOSE(corners3[0].x(), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[0].y(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[1].x(), 6.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[1].y(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[2].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[2].y(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[3].x(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners3[3].y(), 2.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(CenterAndLength) {
  BOOST_CHECK_CLOSE(box3.center_x(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(box3.center_y(), 3, 1e-6);
  BOOST_CHECK_CLOSE(box3.length(), 4, 1e-6);
  BOOST_CHECK_CLOSE(box3.width(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box3.half_length(), 2, 1e-6);
  BOOST_CHECK_CLOSE(box3.half_width(), 1, 1e-6);
  BOOST_CHECK_CLOSE(box3.diagonal(), std::hypot(4.0, 2.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(HasOverlap) {
  BOOST_TEST(!box1.HasOverlap(box2));
  BOOST_TEST(!box1.HasOverlap(box3));
  BOOST_TEST(!box1.HasOverlap(box4));
  BOOST_TEST(!box2.HasOverlap(box4));
  BOOST_TEST(!box3.HasOverlap(box4));
  BOOST_TEST(box4.HasOverlap(box4));

  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(0, 0), Vec2d(1, 1))));
  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(0, 0), Vec2d(3, 3))));
  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(0, -3), Vec2d(0, 3))));
  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(4, 0), Vec2d(-4, 0))));
  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(-4, -4), Vec2d(4, 4))));
  BOOST_TEST(box1.HasOverlap(LineSegment2d(Vec2d(4, -4), Vec2d(-4, 4))));
  BOOST_TEST(!box1.HasOverlap(LineSegment2d(Vec2d(-4, -4), Vec2d(4, -4))));
  BOOST_TEST(!box1.HasOverlap(LineSegment2d(Vec2d(4, -4), Vec2d(4, 4))));
}

BOOST_AUTO_TEST_CASE(GetAABox) {
  AABox2d aabox1 = box1.GetSmallestAABox();
  AABox2d aabox2 = box2.GetSmallestAABox();
  AABox2d aabox3 = box3.GetSmallestAABox();
  AABox2d aabox4 = box4.GetSmallestAABox();

  BOOST_CHECK_CLOSE(aabox1.center_x(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.center_y(), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox1.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox2.center_x(), 5.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.center_y(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox2.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox3.center_x(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.center_y(), 3.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.length(), 4.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox3.width(), 2.0, 1e-6);

  BOOST_CHECK_CLOSE(aabox4.center_x(), 7.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox4.center_y(), 8.0, 1e-6);
  BOOST_CHECK_CLOSE(aabox4.length(), 4.0 * std::sqrt(2.0), 1e-6);
  BOOST_CHECK_CLOSE(aabox4.width(), 4.0 * std::sqrt(2.0), 1e-6);
}

BOOST_AUTO_TEST_CASE(DistanceTo) {
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(3, 0)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(-3, 0)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(0, 2)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(0, -2)), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(0, 0)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(0, 1)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(1, 0)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(0, -1)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(Vec2d(-1, 0)), 0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(LineSegment2d(Vec2d(-4, -4), Vec2d(4, 4))),
                    0.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(LineSegment2d(Vec2d(4, -4), Vec2d(-4, 4))),
                    0.0, 1e-6);

  BOOST_CHECK_CLOSE(box1.DistanceTo(LineSegment2d(Vec2d(0, 2), Vec2d(4, 4))),
                    1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(LineSegment2d(Vec2d(2, 2), Vec2d(3, 1))),
                    std::sqrt(2.0) / 2.0, 1e-6);

  BOOST_CHECK_CLOSE(box1.DistanceTo(box2), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(box1.DistanceTo(box3), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(IsPointIn) {
  BOOST_TEST(box1.IsPointIn(Vec2d(0, 0)));
  BOOST_TEST(box1.IsPointIn(Vec2d(1, 0.5)));
  BOOST_TEST(box1.IsPointIn(Vec2d(-0.5, -1)));
  BOOST_TEST(box1.IsPointIn(Vec2d(2, 1)));
  BOOST_TEST(!box1.IsPointIn(Vec2d(-3, 0)));
  BOOST_TEST(!box1.IsPointIn(Vec2d(0, 2)));
  BOOST_TEST(!box1.IsPointIn(Vec2d(-4, -2)));
}

BOOST_AUTO_TEST_CASE(IsPointOnBoundary) {
  BOOST_TEST(!box1.IsPointOnBoundary(Vec2d(0, 0)));
  BOOST_TEST(!box1.IsPointOnBoundary(Vec2d(1, 0.5)));
  BOOST_TEST(box1.IsPointOnBoundary(Vec2d(-0.5, -1)));
  BOOST_TEST(box1.IsPointOnBoundary(Vec2d(2, 0.5)));
  BOOST_TEST(box1.IsPointOnBoundary(Vec2d(-2, 1)));
  BOOST_TEST(!box1.IsPointOnBoundary(Vec2d(-3, 0)));
  BOOST_TEST(!box1.IsPointOnBoundary(Vec2d(0, 2)));
  BOOST_TEST(!box1.IsPointOnBoundary(Vec2d(-4, -2)));
}

BOOST_AUTO_TEST_CASE(RotateFromCenterAndShift) {
  Box2d box(Vec2d(0, 0), 0, 4, 2);
  BOOST_CHECK_CLOSE(box.heading(), 0.0, 1e-6);
  box.RotateFromCenter(M_PI / 2);
  BOOST_CHECK_CLOSE(box.heading(), M_PI / 2, 1e-6);
  auto corners = box.GetAllCorners();

  BOOST_CHECK_CLOSE(corners[0].x(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[0].y(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].x(), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].y(), 2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].x(), -1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].y(), -2.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].x(), 1.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].y(), -2.0, 1e-6);

  box.Shift({30, 40});
  corners = box.GetAllCorners();

  BOOST_CHECK_CLOSE(corners[0].x(), 31.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[0].y(), 42.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].x(), 29.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[1].y(), 42.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].x(), 29.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[2].y(), 38.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].x(), 31.0, 1e-6);
  BOOST_CHECK_CLOSE(corners[3].y(), 38.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(updatebox) {
  Box2d box_test(Vec2d(0, 0), 0, 4, 2);
  box_test.updatebox2d(1, 2, M_PI / 2);
  auto corners = box_test.GetAllCorners();

  BOOST_CHECK_SMALL(corners[0].x() - 2.0, 1e-6);
  BOOST_CHECK_SMALL(corners[0].y() - 4.0, 1e-6);
  BOOST_CHECK_SMALL(corners[1].x() - 0.0, 1e-6);
  BOOST_CHECK_SMALL(corners[1].y() - 4.0, 1e-6);
  BOOST_CHECK_SMALL(corners[2].x() - 0.0, 1e-6);
  BOOST_CHECK_SMALL(corners[2].y() - 0.0, 1e-6);
  BOOST_CHECK_SMALL(corners[3].x() - 2.0, 1e-6);
  BOOST_CHECK_SMALL(corners[3].y() - 0.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(TESTRANDOM) {
  bool ambiguous = false;

  for (int iter = 0; iter < 10000; ++iter) {
    const double x1 = RandomDouble(-10, 10);
    const double y1 = RandomDouble(-10, 10);
    const double x2 = RandomDouble(-10, 10);
    const double y2 = RandomDouble(-10, 10);
    const double heading1 = RandomDouble(0, M_PI * 2.0);
    const double heading2 = RandomDouble(0, M_PI * 2.0);
    const double l1 = RandomDouble(1, 5);
    const double l2 = RandomDouble(1, 5);
    const double w1 = RandomDouble(1, 5);
    const double w2 = RandomDouble(1, 5);
    const Box2d box1(Vec2d(x1, y1), heading1, l1, w1);
    const Box2d box2(Vec2d(x2, y2), heading2, l2, w2);
    bool overlap = CheckBoxOverlapSlow(box1, box2, &ambiguous);
    if (!ambiguous) {
      BOOST_TEST(overlap == box1.HasOverlap(box2));
    }
  }
}
