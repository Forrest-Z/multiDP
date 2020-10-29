/*
***********************************************************************
* PathSmoothing.h:
* improve the smoothness of path, using conjugate-gradeient descent
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _PATHSMOOTHING_H_
#define _PATHSMOOTHING_H_

#include <iostream>
#include "CollisionChecking.h"

namespace ASV::planning {

class PathSmoothing {
  using vec2d = ASV::common::math::Vec2d;

 public:
  explicit PathSmoothing(const SmootherConfig &smootherconfig)
      : dmax(smootherconfig.d_max) {}
  virtual ~PathSmoothing() = default;

 private:
  const double dmax;
  const double kappa_max;

  std::vector<vec2d> GenerateObstacleTerm(
      const CollisionChecking_Astar &collision_checker,
      const std::vector<vec2d> &path) {
    std::size_t total_num = path.size();
    std::vector<vec2d> obstacleTerm(total_num, vec2d(0, 0));
    auto nearest_obstacles = collision_checker.FindNearstObstacle(path);
    for (std::size_t index = 0; index != total_num; ++index) {
      auto x2o = path[index] - vec2d(nearest_obstacles[index][0],
                                     nearest_obstacles[index][1]);
      // assume the x2o is not very small
      double k = 2 * (1.0 - dmax / x2o.Length());
      obstacleTerm[index] = x2o * k;
    }
    return obstacleTerm;
  }  // GenerateObstacleTerm

  std::vector<vec2d> GenerateCurvatureTerm(const std::vector<vec2d> &path) {
  }  // GenerateCurvatureTerm

  // find the trajectory node when the forward/reverse switch occurs
  void FindForwardReverseSwitch(
      const std::vector<std::tuple<double, double, double, bool>>
          &coarse_trajectory) {}  // FindForwardReverseSwitch

};  // end class PathSmoothing
}  // namespace ASV::planning

#endif /* _PATHSMOOTHING_H_ */