/*
*******************************************************************************
* Smoothing_test.cc:
* unit test for path smoothing
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#include "../include/PathSmoothing.h"

int main() {
  ASV::planning::SmootherConfig smoothconfig{
      1,
  };

  ASV::planning::PathSmoothing pathsmoother(smoothconfig);
}