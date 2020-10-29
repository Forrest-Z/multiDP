/*
*******************************************************************************
* PathPlannerData.h:
* define the data struct used in the path planner
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _PATHPLANNERDATA_H_
#define _PATHPLANNERDATA_H_

#include <array>
#include <common/math/eigen/Eigen/Core>
#include <common/math/eigen/Eigen/Dense>
#include <vector>

namespace ASV::planning {

struct CollisionData {
  /* constraints */
  double MAX_SPEED;      // maximum speed [m/s]
  double MAX_ACCEL;      // maximum acceleration [m/ss]
  double MIN_ACCEL;      // minimum acceleration [m/ss]
  double MAX_ANG_ACCEL;  // maximum angular acceleration [rad/ss]
  double MIN_ANG_ACCEL;  // minimum angular acceleration [rad/ss]
  double MAX_CURVATURE;  // max curvature [1/m]

  /* collision check */
  // box-based
  double HULL_LENGTH;    // [m] Length of vessel hull
  double HULL_WIDTH;     // [m] Width of vessel hull
  double HULL_BACK2COG;  // [m] distance from back edge to CoG
  // circle based
  double ROBOT_RADIUS;  // robot radius[m]
};

struct PathPlannerRTdata {};

}  // namespace ASV::planning

#endif /*_PATHPLANNERDATA_H_*/