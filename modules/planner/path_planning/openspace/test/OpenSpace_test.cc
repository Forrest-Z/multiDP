/*
*******************************************************************************
* OpenSpace_test.cc:
* unit test for openspace planner
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/OpenSpacePlanner.h"
using namespace ASV;

int main() {
  planning::CollisionData _collisiondata{
      4,     // MAX_SPEED
      4.0,   // MAX_ACCEL
      -3.0,  // MIN_ACCEL
      2.0,   // MAX_ANG_ACCEL
      -2.0,  // MIN_ANG_ACCEL
      0.3,   // MAX_CURVATURE
      4,     // HULL_LENGTH
      2,     // HULL_WIDTH
      1,     // HULL_BACK2COG
      3.3    // ROBOT_RADIUS
  };

  planning::OpenSpacePlanner openspace(_collisiondata);
}