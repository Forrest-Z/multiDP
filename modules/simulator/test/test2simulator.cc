/*
***********************************************************************
* testsimulator.cc:
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/simulator.h"
#include "common/plotting/include/gnuplot-iostream.h"

using namespace ASV;

int main() {
  using state_type = Eigen::Matrix<double, 6, 1>;

  common::vessel _vessel{
      (Eigen::Matrix3d() << 100, 0, 1, 0, 100, 0, 1, 0, 1000)
          .finished(),          // Mass
      Eigen::Matrix3d::Zero(),  // AddedMass
      (Eigen::Matrix3d() << 100, 0, 0, 0, 200, 0, 0, 0, 300)
          .finished(),          // LinearDamping
      Eigen::Matrix3d::Zero(),  // QuadraticDamping
      Eigen::Vector3d::Zero(),  // cog
      Eigen::Vector2d::Zero(),  // x_thrust
      Eigen::Vector2d::Zero(),  // y_thrust
      Eigen::Vector2d::Zero(),  // mz_thrust
      Eigen::Vector2d::Zero(),  // surge_v
      Eigen::Vector2d::Zero(),  // sway_v
      Eigen::Vector2d::Zero(),  // yaw_v
      Eigen::Vector2d::Zero(),  // roll_v
      0,                        // L
      0                         // B
  };

  state_type x = (state_type() << 3, 5, 0.5, 0, 0, 0).finished();
  state_type x2 = (state_type() << 5, 3, -0.5, 0, 0, 0).finished();
  Eigen::Matrix3d P =
      (Eigen::Matrix3d() << 10, 0, 0, 0, 10, 0, 0, 0, 100).finished();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  Eigen::Vector3d u2 = Eigen::Vector3d::Zero();

  int total_step = 5000;
  Eigen::MatrixXd save_x(6, total_step);
  Eigen::MatrixXd save_x_2(6, total_step);

  simulation::simulator _simulator(0.1, _vessel, x);
  simulation::simulator _simulator2(0.1, _vessel, x2);

  for (int i = 0; i != total_step; ++i) {
    u = -P * x.head(3);
    u2 = -P * x2.head(3);
    x = _simulator.simulator_onestep(0, u).getX();
    x2 = _simulator2.simulator_onestep(0, u2).getX();
    save_x.col(i) = x;
    save_x_2.col(i) = x2;
  }

  Gnuplot gp;
  std::vector<std::pair<double, double> > xy_pts_A;
  gp << "set terminal x11 size 800, 1000 0\n";
  gp << "set multiplot layout 3, 1 title 'Simulation results' font ',14'\n";
  std::vector<std::string> label_names = {"X", "Y", "theta"};
  for (int i = 0; i != 3; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel '" << label_names[i] << "'\n";
    gp << "plot"
          " '-' with lines lt 1 lw 3 lc rgb 'violet' \n";
    xy_pts_A.clear();
    for (int j = 0; j != total_step; ++j) {
      xy_pts_A.push_back(std::make_pair(j, save_x(i, j)));
    }
    gp.send1d(xy_pts_A);
  }
  gp << "unset multiplot\n";

  Gnuplot gp2;
  std::vector<std::pair<double, double> > xy_pts_A2;
  gp2 << "set terminal x11 size 800, 1000 0\n";
  gp2 << "set multiplot layout 3, 1 title 'Simulation results' font ',14'\n";
  std::vector<std::string> label_names2 = {"z", "Y", "theta"};
  for (int i = 0; i != 3; ++i) {
    gp2 << "set xtics out\n";
    gp2 << "set ytics out\n";
    gp2 << "set ylabel '" << label_names2[i] << "'\n";
    gp2 << "plot"
           " '-' with lines lt 1 lw 3 lc rgb 'violet' \n";
    xy_pts_A2.clear();
    for (int j = 0; j != total_step; ++j) {
      xy_pts_A2.push_back(std::make_pair(j, save_x_2(i, j)));
    }
    gp2.send1d(xy_pts_A2);
  }
  gp2 << "unset multiplot\n";
}