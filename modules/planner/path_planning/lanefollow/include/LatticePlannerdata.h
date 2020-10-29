/*
****************************************************************************
* LatticePlannerdata.h:
* define the data struct used in the controller
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _LATTICEPLANNERDATA_H_
#define _LATTICEPLANNERDATA_H_

#include "modules/planner/path_planning/common/PathPlannerData.h"

namespace ASV::planning {

// state in the Cartesian coodinate
struct CartesianState {
  double x;
  double y;
  double theta;
  double kappa;
  double speed;      // vx
  double dspeed;     // ax
  double yaw_rate;   // rad/s
  double yaw_accel;  // rad/s^2
};

// state in the Frenet coodinate
struct FrenetState {
  double s;         // s: longitudinal coordinate w.r.t reference line.
  double s_dot;     // ds / dt
  double s_ddot;    // d(s_dot) / dt
  double d;         // d: lateral coordinate w.r.t. reference line
  double d_dot;     // dd / dt
  double d_ddot;    // d(d_dot) / dt
  double d_prime;   // dd / ds
  double d_pprime;  // d(d_prime)/ ds
};

struct Frenet_path {
  Eigen::VectorXd t;  // time (s)
  Eigen::VectorXd d;  // lateral error (m)
  Eigen::VectorXd d_dot;
  Eigen::VectorXd d_ddot;
  Eigen::VectorXd s;  // longitudual error (m)
  Eigen::VectorXd s_dot;
  Eigen::VectorXd s_ddot;
  Eigen::VectorXd d_prime;
  Eigen::VectorXd d_pprime;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd yaw;
  Eigen::VectorXd kappa;
  Eigen::VectorXd speed;
  Eigen::VectorXd dspeed;
  Eigen::VectorXd yaw_rate;   // rad/s
  Eigen::VectorXd yaw_accel;  // rad/s^2
  double cd;
  double cv;
  double cf;
};

struct LatticeData {
  double SAMPLE_TIME;  //[s]
  double MAX_SPEED;    //[m/s]
  /* end condition of Frenet Lattics */
  double TARGET_COURSE_ARC_STEP;  //[m]
  double MAX_ROAD_WIDTH;          // max lateral deviation [m]
  double ROAD_WIDTH_STEP;         // road width sampling length [m]
  double MAXT;                    // max prediction time [s]
  double MINT;                    // min prediction time [s]
  double DT;                      // time tick [s]
  double MAX_SPEED_DEVIATION;     // Max speed deviation [m/s]
  double TRAGET_SPEED_STEP;       // target speed sampling length [m/s]
};

}  // namespace ASV::planning

#endif /*_LATTICEPLANNERDATA_H_*/
