/*
*******************************************************************************
* OpenSpacePlanner.h:
* Path planner used in the low-speed vessel, including hybrid A star,
* trajectory smoother and collision checking. This planner is designed to
* be used in both fully-actuated and underactuated vessels.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _OPENSPACEPLANNER_H_
#define _OPENSPACEPLANNER_H_

#include "HybridAStar.h"

namespace ASV::planning {

class OpenSpacePlanner {
 public:
  OpenSpacePlanner(const CollisionData &collisiondata)
      : collision_checker_(collisiondata) {}
  virtual ~OpenSpacePlanner() = default;

  OpenSpacePlanner &update_obstacles(
      const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
      const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment,
      const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
    collision_checker_.set_all_obstacls(Obstacles_Vertex, Obstacles_LineSegment,
                                        Obstacles_Box2d);

    return *this;
  }  // update_obstacles

 private:
  CollisionChecking_Astar collision_checker_;

};  // end class OpenSpacePlanner

}  // namespace ASV::planning

#endif /* _OPENSPACEPLANNER_H_ */