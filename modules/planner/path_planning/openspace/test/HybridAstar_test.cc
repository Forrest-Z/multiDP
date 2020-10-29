/////////////////////////////////////////////////////////////////////////

// Hybrid A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "../include/HybridAStar.h"
#include <chrono>
#include <iostream>
#include <thread>
#include "common/plotting/include/gnuplot-iostream.h"
using namespace ASV;

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

double ego_length = _collisiondata.HULL_LENGTH;
double ego_width = _collisiondata.HULL_WIDTH;
double ego_center_local_x = 0.5 * ego_length - _collisiondata.HULL_BACK2COG;
double ego_center_local_y = 0.0;

void plot_vessel(Gnuplot &_gp, const std::array<double, 3> &vessel_state,
                 const int id) {
  double cvalue = std::cos(vessel_state.at(2));
  double svalue = std::sin(vessel_state.at(2));
  double ego_center_x = vessel_state.at(0) + cvalue * ego_center_local_x -
                        svalue * ego_center_local_y;
  double ego_center_y = vessel_state.at(1) + svalue * ego_center_local_x +
                        cvalue * ego_center_local_y;

  common::math::Box2d ego_box(
      ASV::common::math::Vec2d(ego_center_x, ego_center_y), vessel_state.at(2),
      ego_length, ego_width);

  auto allcorners = ego_box.GetAllCorners();

  //
  _gp << "set object " + std::to_string(id) + " polygon from";
  for (int j = 0; j != 4; ++j) {
    _gp << " " << allcorners[j].x() << ", " << allcorners[j].y() << " to";
  }
  _gp << " " << allcorners[0].x() << ", " << allcorners[0].y() << "\n";
  _gp << "set object " + std::to_string(id) +
             " fc rgb 'blue' fillstyle solid 0.1 noborder\n";
}

// illustrate the hybrid A* planner at a time instant
void rtplotting_4dbestpath(
    Gnuplot &_gp, const std::array<double, 3> &start_point,
    const std::array<double, 3> &end_point,
    const std::tuple<double, double, double, bool> &state,
    const std::vector<std::tuple<double, double, double, bool>> &trajectory,
    const std::vector<planning::Obstacle_Vertex_Config> &Obstacles_Vertex,
    const std::vector<planning::Obstacle_LineSegment_Config>
        &Obstacles_LineSegment,
    const std::vector<planning::Obstacle_Box2d_Config> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  plot_vessel(_gp, start_point, 1);
  plot_vessel(_gp, end_point, 2);
  plot_vessel(_gp, {std::get<0>(state), std::get<1>(state), std::get<2>(state)},
              3);

  // obstacle (box)
  for (std::size_t i = 0; i != Obstacles_Box2d.size(); ++i) {
    ASV::common::math::Box2d ob_box(
        ASV::common::math::Vec2d(Obstacles_Box2d[i].center_x,
                                 Obstacles_Box2d[i].center_y),
        Obstacles_Box2d[i].heading, Obstacles_Box2d[i].length,
        Obstacles_Box2d[i].width);
    auto allcorners = ob_box.GetAllCorners();

    //
    _gp << "set object " + std::to_string(i + 5) + " polygon from";
    for (int j = 0; j != 4; ++j) {
      _gp << " " << allcorners[j].x() << ", " << allcorners[j].y() << " to";
    }
    _gp << " " << allcorners[0].x() << ", " << allcorners[0].y() << "\n";
    _gp << "set object " + std::to_string(i + 5) +
               " fc rgb '#000000' fillstyle solid lw 0\n";
  }

  _gp << "plot ";
  // trajectory
  for (const auto &ps : trajectory) {
    xy_pts_A.push_back(std::make_pair(std::get<0>(ps), std::get<1>(ps)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'best path',";

  // obstacle (vertex)
  xy_pts_A.clear();
  for (std::size_t i = 0; i != Obstacles_Vertex.size(); ++i) {
    xy_pts_A.push_back(
        std::make_pair(Obstacles_Vertex[i].x, Obstacles_Vertex[i].y));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with points pt 9 ps 3 lc rgb '#000000' title 'obstacles_v', ";

  // obstacle (Line Segment)
  for (std::size_t i = 0; i != Obstacles_LineSegment.size(); ++i) {
    xy_pts_A.clear();
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].start_x,
                                      Obstacles_LineSegment[i].start_y));
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].end_x,
                                      Obstacles_LineSegment[i].end_y));
    _gp << _gp.file1d(xy_pts_A)
        << " with linespoints linetype 1 lw 2 lc rgb '#000000' pointtype 7 "
           "pointsize 1 notitle, ";
  }
  _gp << "\n";

  _gp.flush();

}  // rtplotting_4dbestpath

// illustrate the hybrid A* planner at a time instant
void rtplotting_2dbestpath(
    Gnuplot &_gp, const std::array<double, 3> &start_point,
    const std::array<double, 3> &end_point, const std::array<double, 3> &state,
    const std::vector<std::array<double, 3>> &trajectory,
    const std::vector<planning::Obstacle_Vertex_Config> &Obstacles_Vertex,
    const std::vector<planning::Obstacle_LineSegment_Config>
        &Obstacles_LineSegment,
    const std::vector<planning::Obstacle_Box2d_Config> &Obstacles_Box2d) {
  std::vector<std::pair<double, double>> xy_pts_A;

  plot_vessel(_gp, start_point, 1);
  plot_vessel(_gp, end_point, 2);
  plot_vessel(_gp, state, 3);

  // obstacle (box)
  for (std::size_t i = 0; i != Obstacles_Box2d.size(); ++i) {
    ASV::common::math::Box2d ob_box(
        ASV::common::math::Vec2d(Obstacles_Box2d[i].center_x,
                                 Obstacles_Box2d[i].center_y),
        Obstacles_Box2d[i].heading, Obstacles_Box2d[i].length,
        Obstacles_Box2d[i].width);
    auto allcorners = ob_box.GetAllCorners();

    //
    _gp << "set object " + std::to_string(i + 5) + " polygon from";
    for (int j = 0; j != 4; ++j) {
      _gp << " " << allcorners[j].x() << ", " << allcorners[j].y() << " to";
    }
    _gp << " " << allcorners[0].x() << ", " << allcorners[0].y() << "\n";
    _gp << "set object " + std::to_string(i + 5) +
               " fc rgb '#000000' fillstyle solid lw 0\n";
  }

  _gp << "plot ";
  // trajectory
  for (const auto &ps : trajectory) {
    xy_pts_A.push_back(std::make_pair(ps.at(0), ps.at(1)));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
         "pointsize 1 title 'best path',";

  // obstacle (vertex)
  xy_pts_A.clear();
  for (std::size_t i = 0; i != Obstacles_Vertex.size(); ++i) {
    xy_pts_A.push_back(
        std::make_pair(Obstacles_Vertex[i].x, Obstacles_Vertex[i].y));
  }
  _gp << _gp.file1d(xy_pts_A)
      << " with points pt 9 ps 3 lc rgb '#000000' title 'obstacles_v', ";

  // obstacle (Line Segment)
  for (std::size_t i = 0; i != Obstacles_LineSegment.size(); ++i) {
    xy_pts_A.clear();
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].start_x,
                                      Obstacles_LineSegment[i].start_y));
    xy_pts_A.push_back(std::make_pair(Obstacles_LineSegment[i].end_x,
                                      Obstacles_LineSegment[i].end_y));
    _gp << _gp.file1d(xy_pts_A)
        << " with linespoints linetype 1 lw 2 lc rgb '#000000' pointtype 7 "
           "pointsize 1 notitle, ";
  }
  _gp << "\n";

  _gp.flush();

}  // rtplotting_2dbestpath

// generate map and endpoint
void generate_obstacle_map(
    std::vector<planning::Obstacle_Vertex_Config> &Obstacles_Vertex,
    std::vector<planning::Obstacle_LineSegment_Config> &Obstacles_LS,
    std::vector<planning::Obstacle_Box2d_Config> &Obstacles_Box,
    std::array<float, 3> &start_point, std::array<float, 3> &end_point,
    int type) {
  // type:
  // 0: parking lot
  //

  switch (type) {
    case 0:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          10,  // start_x
          10,  // start_y
          10,  // end_x
          20   // end_y
      });
      Obstacles_LS.push_back({
          10,   // start_x
          20,   // start_y
          -10,  // end_x
          20    // end_y
      });
      Obstacles_LS.push_back({
          -10,  // start_x
          20,   // start_y
          -10,  // end_x
          10    // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // start point
      start_point = {0, 0, 0.5 * M_PI};
      // end point
      end_point = {20, 0, 0.0 * M_PI};
      break;
    case 1:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          1.5,  // start_x
          0,    // start_y
          1.5,  // end_x
          5     // end_y
      });
      Obstacles_LS.push_back({
          1.5,   // start_x
          5,     // start_y
          -1.5,  // end_x
          5      // end_y
      });
      Obstacles_LS.push_back({
          -1.5,  // start_x
          5,     // start_y
          -1.5,  // end_x
          0      // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });

      // start point
      start_point = {-20, -20, 0.0 * M_PI};
      // end point
      end_point = {0, 1.5, 0.5 * M_PI};
      break;
    case 2:
      // vertex
      Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          4,   // start_y
          24,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          8,  // start_x
          4,  // start_y
          8,  // end_x
          -1  // end_y
      });
      Obstacles_LS.push_back({
          24,  // start_x
          8,   // start_y
          24,  // end_x
          2    // end_y
      });
      Obstacles_LS.push_back({
          24,  // start_x
          -8,  // start_y
          24,  // end_x
          -2   // end_y
      });
      Obstacles_LS.push_back({
          0,   // start_x
          -4,  // start_y
          20,  // end_x
          -4   // end_y
      });
      Obstacles_LS.push_back({
          16,  // start_x
          -4,  // start_y
          16,  // end_x
          1    // end_y
      });
      // box
      Obstacles_Box.push_back({
          10,  // center_x
          10,  // center_y
          4,   // length
          1,   // width
          0    // heading
      });
      // start point
      start_point = {0, 0, 0.0 * M_PI};

      // end point
      end_point = {26, 0.5, 0.0 * M_PI};
      break;
    case 3:
      // vertex
      // Obstacles_Vertex.push_back({10, 4});
      // linesegment
      Obstacles_LS.push_back({
          20,  // start_x
          25,  // start_y
          20,  // end_x
          40   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          25,  // start_y
          23,  // end_x
          25   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          40,  // start_y
          40,  // end_x
          35   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          55,  // start_y
          40,  // end_x
          50   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          55,  // start_y
          20,  // end_x
          65   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          65,  // start_y
          40,  // end_x
          60   // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          60,  // start_y
          40,  // end_x
          50   // end_y
      });
      Obstacles_LS.push_back({
          40,  // start_x
          55,  // start_y
          80,  // end_x
          55   // end_y
      });
      Obstacles_LS.push_back({
          100,  // start_x
          70,   // start_y
          90,   // end_x
          70    // end_y
      });
      Obstacles_LS.push_back({
          90,  // start_x
          30,  // start_y
          90,  // end_x
          70   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          95,  // end_x
          76   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          38,  // end_x
          70   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          70,  // start_y
          20,  // end_x
          73   // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          73,  // start_y
          20,  // end_x
          80   // end_y
      });
      Obstacles_LS.push_back({
          38,  // start_x
          76,  // start_y
          20,  // end_x
          80   // end_y
      });
      // box
      Obstacles_Box.push_back({
          4,          // center_x
          7.5,        // center_y
          15,         // length
          8,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          19,         // center_x
          7.5,        // center_y
          15,         // length
          8,          // width
          0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,         // center_x
          12.5,       // center_y
          55,         // length
          5,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          45.5,       // center_x
          2,          // center_y
          15,         // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          5,          // center_x
          35,         // center_y
          10,         // length
          20,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          3.5,        // center_x
          55,         // center_y
          15,         // length
          7,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          58,         // center_x
          30,         // center_y
          70,         // length
          10,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          83,         // center_x
          55,         // center_y
          7,          // length
          10,         // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          70,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          65,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          56,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          53,          // center_x
          57,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          50,          // center_x
          53,          // center_y
          4,           // length
          3,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          53,          // center_y
          4,           // length
          3,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          44,          // center_x
          52,          // center_y
          6,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          67,          // center_x
          53,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          23,         // center_x
          42,         // center_y
          5,          // length
          3,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          30,         // center_x
          40,         // center_y
          4,          // length
          2,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          34,         // center_x
          39,         // center_y
          4,          // length
          2,          // width
          0.4 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          70,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          60,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          40,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          50,          // center_x
          37,          // center_y
          4,           // length
          2,           // width
          -0.5 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          88,         // center_x
          38,         // center_y
          3,          // length
          6,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          48,         // center_x
          74,         // center_y
          20,         // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          68,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          78,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          88,         // center_x
          74,         // center_y
          2,          // length
          4,          // width
          0.0 * M_PI  // heading
      });
      Obstacles_Box.push_back({
          10,         // center_x
          74,         // center_y
          4,          // length
          4,          // width
          0.3 * M_PI  // heading
      });
      // start point
      start_point = {5, 20, 0.0 * M_PI};

      // end point
      end_point = {76, 60, -0.5 * M_PI};
      break;

    case 4:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,    // start_x
          3,    // start_y
          9.5,  // end_x
          3     // end_y
      });
      Obstacles_LS.push_back({
          9.5,  // start_x
          3,    // start_y
          9.5,  // end_x
          0     // end_y
      });
      Obstacles_LS.push_back({
          9.5,   // start_x
          0,     // start_y
          15.5,  // end_x
          0      // end_y
      });
      Obstacles_LS.push_back({
          15.5,  // start_x
          0,     // start_y
          15.5,  // end_x
          3      // end_y
      });
      Obstacles_LS.push_back({
          15.5,  // start_x
          3,     // start_y
          30,    // end_x
          3      // end_y
      });
      Obstacles_LS.push_back({
          0,   // start_x
          10,  // start_y
          30,  // end_x
          10   // end_y
      });
      // start point
      start_point = {4, 6, 0.3 * M_PI};
      // end point
      end_point = {11.5, 1.5, 0 * M_PI};
      break;

    case 5:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          4,   // start_y
          20,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          4,   // start_y
          20,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          0,   // start_y
          23,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          0,   // start_y
          23,  // end_x
          4    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          4,   // start_y
          40,  // end_x
          4    // end_y
      });
      // start point
      start_point = {30, 7, 0.0 * M_PI};
      // end point
      end_point = {21.5, 1.5, 0.5 * M_PI};
      break;

    case 6:
      // vertex
      Obstacles_Vertex.push_back({-10, 4});
      // linesegment
      Obstacles_LS.push_back({
          0,   // start_x
          5,   // start_y
          20,  // end_x
          5    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          5,   // start_y
          20,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          20,  // start_x
          0,   // start_y
          23,  // end_x
          0    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          0,   // start_y
          23,  // end_x
          5    // end_y
      });
      Obstacles_LS.push_back({
          23,  // start_x
          5,   // start_y
          40,  // end_x
          5    // end_y
      });
      // start point
      start_point = {30, 7, 0.0 * M_PI};
      // end point
      end_point = {21.5, 3.5, -0.5 * M_PI};
      break;
    default:
      break;
  };

}  // generate_obstacle_map

// Main
int main() {
  planning::HybridAStarConfig _HybridAStarConfig{
      1,    // move_length
      1.5,  // penalty_turning
      2,    // penalty_reverse
      2     // penalty_switch
            // 5,    // num_interpolate
  };

  int test_scenario = 6;

  // obstacles
  std::vector<planning::Obstacle_Vertex_Config> Obstacles_Vertex;
  std::vector<planning::Obstacle_LineSegment_Config> Obstacles_LS;
  std::vector<planning::Obstacle_Box2d_Config> Obstacles_Box;
  std::array<float, 3> start_point;
  std::array<float, 3> end_point;
  generate_obstacle_map(Obstacles_Vertex, Obstacles_LS, Obstacles_Box,
                        start_point, end_point, test_scenario);

  planning::HybridAStar Hybrid_AStar(_collisiondata, _HybridAStarConfig);

  ASV::planning::CollisionChecking_Astar collision_checker_(_collisiondata);
  collision_checker_.set_all_obstacls(Obstacles_Vertex, Obstacles_LS,
                                      Obstacles_Box);

  Hybrid_AStar.setup_start_end(start_point.at(0), start_point.at(1),
                               start_point.at(2), end_point.at(0),
                               end_point.at(1), end_point.at(2));

  Hybrid_AStar.setup_2d_start_end(start_point.at(0), start_point.at(1),
                                  start_point.at(2), end_point.at(0),
                                  end_point.at(1), end_point.at(2));

  Hybrid_AStar.perform_4dnode_search(collision_checker_);
  Hybrid_AStar.perform_2dnode_search(collision_checker_);

  auto hr = Hybrid_AStar.hybridastar_trajecotry();
  auto hr2d = Hybrid_AStar.hybridastar_2dtrajecotry();
  // std::vector<std::array<double, 3>> hr = {{1, 20, 0}};
  // plotting
  Gnuplot gp;
  gp << "set terminal x11 size 1100, 1100 0\n";
  gp << "set title 'A star search (4d)'\n";
  gp << "set xrange [0:100]\n";
  gp << "set yrange [0:100]\n";
  gp << "set size ratio -1\n";

  for (std::size_t i = 0; i != hr.size(); ++i) {
    std::cout << std::get<0>(hr[i]) << ", " << std::get<1>(hr[i]) << ", "
              << std::get<2>(hr[i]) << ", " << std::get<3>(hr[i]) << std::endl;
    rtplotting_4dbestpath(
        gp, {start_point.at(0), start_point.at(1), start_point.at(2)},
        {end_point.at(0), end_point.at(1), end_point.at(2)}, hr[i], hr,
        Obstacles_Vertex, Obstacles_LS, Obstacles_Box);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Gnuplot gp1;
  // gp1 << "set terminal x11 size 1100, 1100 1\n";
  // gp1 << "set title 'A star search (2d)'\n";
  // gp1 << "set xrange [0:100]\n";
  // gp1 << "set yrange [0:100]\n";
  // gp1 << "set size ratio -1\n";

  // for (std::size_t i = 0; i != hr2d.size(); ++i) {
  //   rtplotting_2dbestpath(
  //       gp1, {start_point.at(0), start_point.at(1), start_point.at(2)},
  //       {end_point.at(0), end_point.at(1), end_point.at(2)}, hr2d[i], hr2d,
  //       Obstacles_Vertex, Obstacles_LS, Obstacles_Box);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }

  return 0;
}