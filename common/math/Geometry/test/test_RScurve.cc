/*
***********************************************************************
* test_RScurve.cc:
* Utility test for Reeds Shepp curves
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <iostream>
#include "../../../plotting/include/gnuplot-iostream.h"
#include "../include/Reeds_Shepp.h"

void rtplotting(Gnuplot &_gp, const double *state,
                const std::vector<std::array<double, 3>> &allstate) {
  static double length_ = 0.5;

  double arrow_start_x = state[0];
  double arrow_start_y = state[1];

  double angle = state[2];
  double arrow_end_x = arrow_start_x + length_ * std::cos(angle);
  double arrow_end_y = arrow_start_y + length_ * std::sin(angle);
  _gp << "set arrow from " << arrow_start_x << ", " << arrow_start_y << " to "
      << arrow_end_x << ", " << arrow_end_y << "as 1 \n";

  std::vector<std::pair<double, double>> xy_pts_C;

  for (unsigned int i = 0; i != allstate.size(); ++i) {
    xy_pts_C.push_back(std::make_pair(allstate[i][0], allstate[i][1]));
  }
  // the second subplot
  _gp << "plot " << _gp.file1d(xy_pts_C)
      << " with lines lt 1 lw 2 lc rgb 'red' notitle\n";

  _gp << "unset arrow 1\n";

  _gp.flush();
}  // rtplotting

int main() {
  std::array<double, 3> q0 = {2, 2, 0.0 * M_PI};
  std::array<double, 3> q1 = {-6, 8, 1 * M_PI};

  ASV::common::math::ReedsSheppStateSpace r(3);

  //----------------------------get curve type-------------------------
  auto RStypes = r.rs_type(q0, q1);
  std::cout << "rscurve type\n";
  for (unsigned int i = 0; i < RStypes.size(); i++) {
    std::cout << RStypes[i] << std::endl;
  }

  //--------------------q0 to q1 discrete point-----------------
  auto finalpath = r.rs_state(q0, q1, 0.1);

  //
  auto switch_test = r.rs_trajectory(q0, q1, 0.1);

  for (const auto &value : switch_test) {
    std::cout << std::get<0>(value) << ", " << std::get<1>(value) << ", "
              << std::get<2>(value) << ", " << std::get<3>(value) << std::endl;
  }
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 1000 1\n";
  gp << "set grid \n";
  gp << "set size ratio -1\n";
  gp << "set title 'Reeds-Shepp curve'\n";
  gp << "set xlabel 'X (m)'\n";
  gp << "set ylabel 'Y (m)'\n";
  gp << "set style arrow 1 head filled size screen 0.025,30,45 ls 1\n";

  for (unsigned int i = 0; i < finalpath.size(); i++) {
    double state[3] = {finalpath[i][0], finalpath[i][1], finalpath[i][2]};
    rtplotting(gp, state, finalpath);
    std::cout << finalpath[i][0] << " " << finalpath[i][1] << " "
              << finalpath[i][2] << std::endl;
  }
}
