/*
*******************************************************************************
* openspacedata.h:
* define the data struct used in the low-speed planner
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _OPENSPACEDATA_H_
#define _OPENSPACEDATA_H_

#include "common/math/Geometry/include/box2d.h"
#include "modules/planner/path_planning/common/PathPlannerData.h"

namespace ASV::planning {

struct HybridAStarConfig {
  float move_length;      // length of each movement
  float penalty_turning;  // penalty of turning
  float penalty_reverse;  // penalty of reverse
  float penalty_switch;   // penalty of direction switch
  // unsigned num_interpolate;  // # of interpolation of each movement
};

// parameters used in search algorithm (Hybrid A star)
struct SearchConfig {
  float move_length;    // length of each movement
  float turning_angle;  // angle of max turning
  float cost_map[6][6];
};

// in the Cartesian Coordinates.
struct OpenSpace_Trajectory {
  Eigen::VectorXd t;      // time (s)
  Eigen::VectorXd x;      // x (m)
  Eigen::VectorXd y;      // y (m)
  Eigen::VectorXd theta;  // heading (rad)
  Eigen::VectorXd speed;  // speed (rad/s)
};

struct SmootherConfig {
  double d_max;
};

/**************************** obstacles  ******************************/
// the obstacles in openspace planner can be defined in different forms,
// including vertex, linesegment and box. These parameters are represented
// in the Cartesian Coordinates.
struct Obstacle_Vertex_Config {
  double x;
  double y;
};

struct Obstacle_LineSegment_Config {
  double start_x;  // x-coordinate of starting point
  double start_y;  // y-coordinate of starting point
  double end_x;    // x-coordinate of end point
  double end_y;    // y-coordinate of end point
};

struct Obstacle_Box2d_Config {
  double center_x;  // x-coordinate of box center
  double center_y;  // y-coordinate of box center
  double length;    // length, parellel to heading-axis
  double width;     // width, perpendicular to heading-axis
  double heading;
};

template <std::size_t max_num>
struct Obstacle_Vertex {
  std::array<bool, max_num> status;
  std::array<ASV::common::math::Vec2d, max_num> vertex;
};

template <std::size_t max_num>
struct Obstacle_LineSegment {
  std::array<bool, max_num> status;
  std::array<ASV::common::math::LineSegment2d, max_num> linesegment;
};

template <std::size_t max_num>
struct Obstacle_Box2d {
  std::array<bool, max_num> status;
  std::array<ASV::common::math::Box2d, max_num> box2d;
};

}  // namespace ASV::planning
#endif /* _OPENSPACEDATA_H_ */