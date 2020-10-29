/*
*******************************************************************************
* CollisionChecking_test.cc:
* unit test for collision checking
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/CollisionChecking.h"
#include "common/timer/include/timecounter.h"

#include <iostream>
using namespace ASV;

int main() {
  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      2.0,   // MAX_ANG_ACCEL
      -2.0,  // MIN_ANG_ACCEL
      0.2,   // MAX_CURVATURE
      4,     // HULL_LENGTH
      2,     // HULL_WIDTH
      1,     // HULL_BACK2COG
      3.3    // ROBOT_RADIUS
  };
  std::vector<planning::Obstacle_Vertex_Config> Obstacles_Vertex{{
      10,  // x
      4    // y
  }};
  std::vector<planning::Obstacle_LineSegment_Config> Obstacles_lS{
      {
          4,  // start_x
          4,  // start_y
          4,  // end_x
          8   // end_y
      },
      {
          4,   // start_x
          8,   // start_y
          -4,  // end_x
          8    // end_y
      },
      {
          -4,  // start_x
          8,   // start_y
          -4,  // end_x
          4    // end_y
      }};
  std::vector<planning::Obstacle_Box2d_Config> Obstacles_Box{{
      10,  // center_x
      10,  // center_y
      4,   // length
      1,   // width
      0    // heading
  }};

  common::timecounter _timer;

  Eigen::VectorXd x(5);
  Eigen::VectorXd y(5);
  Eigen::VectorXd theta(5);

  std::vector<std::array<double, 3>> _OpenSpace_Trajectory;
  x << 0, 1.1, 2.0, 3.4, 4.5;
  y << 0, -3.1, 10, 20, 11.3;
  theta << 0, 0, 0, 8.7e-8, 0;

  for (int i = 0; i != x.size(); ++i)
    _OpenSpace_Trajectory.push_back({x(i), y(i), theta(i)});

  planning::CollisionChecking<> _CollisionChecking(_collisiondata);
  _CollisionChecking.set_all_obstacls(Obstacles_Vertex, Obstacles_lS,
                                      Obstacles_Box);

  _timer.timeelapsed();

  for (int i = 0; i != 100; i++)
    if (_CollisionChecking.InCollision(_OpenSpace_Trajectory))
      std::cout << "collison occur!\n";
    else
      std::cout << "collison free\n";

  long int et = _timer.timeelapsed();
  std::cout << "elapsed time of collision checking: " << et << std::endl;

  auto nearst_results =
      _CollisionChecking.FindNearstObstacle(_OpenSpace_Trajectory);

  et = _timer.timeelapsed();

  std::cout << "elapsed time of nearest point: " << et << std::endl;

  for (const auto &nearst_result : nearst_results)
    std::cout << nearst_result[0] << " " << nearst_result[1] << std::endl;
}
