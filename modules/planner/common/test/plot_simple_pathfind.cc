/////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "../include/stlastar.h"
#include "common/plotting/include/gnuplot-iostream.h"

#include <math.h>
#include <stdio.h>
#include <iostream>

constexpr int DEBUG_LISTS = 0;
constexpr int DEBUG_LIST_LENGTHS_ONLY = 0;

// Global data

// The world map

const int MAP_WIDTH = 20;
const int MAP_HEIGHT = 30;

int world_map[MAP_WIDTH * MAP_HEIGHT] = {

    // 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 00
    1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1,  // 01
    1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 02
    1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 03
    1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,  // 04
    1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 05
    1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,  // 06
    1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,  // 07
    1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 08
    1, 9, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,  // 09
    1, 9, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 10
    1, 9, 9, 9, 9, 9, 1, 9, 1, 9, 1, 9, 9, 9, 9, 9, 1, 1, 1, 1,  // 11
    1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 12
    1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 13
    1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,  // 14
    1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 15
    1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,  // 16
    1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 1, 9, 9, 9, 9,  // 17
    1, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 18
    1, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 9, 1,  // 19
    1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 1,  // 20
    1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 21
    9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1,  // 22
    1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1,  // 23
    1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 24
    1, 9, 9, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 25
    1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 1,  // 26
    1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 27
    1, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 1, 9, 1,  // 28
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 29

};

// map helper functions

int GetMap(int x, int y) {
  if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
    return 9;
  }

  return world_map[(y * MAP_WIDTH) + x];
}

// Definitions

class MapSearchNode {
 public:
  int x;  // the (x,y) positions of the node
  int y;

  MapSearchNode() : x(0), y(0) {}
  MapSearchNode(int px, int py) : x(px), y(py) {}
  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(MapSearchNode &nodeGoal) {
    return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
  }  // GoalDistanceEstimate

  bool IsGoal(MapSearchNode &nodeGoal) {
    if ((x == nodeGoal.x) && (y == nodeGoal.y)) {
      return true;
    }

    return false;
  }  // IsGoal

  // This generates the successors to the given Node. It uses a helper function
  // called AddSuccessor to give the successors to the AStar class. The A*
  // specific initialisation is done for each node internally, so here you just
  // set the state information that is specific to the application
  bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch,
                     MapSearchNode *parent_node) {
    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
      parent_x = parent_node->x;
      parent_y = parent_node->y;
    }

    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if ((GetMap(x - 1, y) < 9) && !((parent_x == x - 1) && (parent_y == y))) {
      NewNode = MapSearchNode(x - 1, y);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y - 1) < 9) && !((parent_x == x) && (parent_y == y - 1))) {
      NewNode = MapSearchNode(x, y - 1);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x + 1, y) < 9) && !((parent_x == x + 1) && (parent_y == y))) {
      NewNode = MapSearchNode(x + 1, y);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y + 1) < 9) && !((parent_x == x) && (parent_y == y + 1))) {
      NewNode = MapSearchNode(x, y + 1);
      astarsearch->AddSuccessor(NewNode);
    }

    return true;
  }  // GetSuccessors

  // given this node, what does it cost to move to successor. In the case
  // of our map the answer is the map terrain value at this node since that is
  // conceptually where we're moving
  float GetCost(MapSearchNode &successor) {
    // return (float)GetMap(x, y);
    return 0;
  }  // GetCost

  bool IsSameState(MapSearchNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((x == rhs.x) && (y == rhs.y)) {
      return true;
    } else {
      return false;
    }
  }  // IsSameState

  void PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x, y);

    std::cout << str;
  }  // PrintNodeInfo
};

// Main
int main() {
  std::cout << "STL A* Search implementation\n";

  // Our sample problem defines the world as a 2d array representing a terrain
  // Each element contains an integer from 0 to 5 which indicates the cost
  // of travel across the terrain. Zero means the least possible difficulty
  // in travelling (think ice rink if you can skate) whilst 5 represents the
  // most difficult. 9 indicates that we cannot pass.

  // Create an instance of the search class...
  AStarSearch<MapSearchNode> astarsearch;

  // Create a start state
  MapSearchNode nodeStart;
  nodeStart.x = 5;
  nodeStart.y = 6;

  // Define the goal state
  MapSearchNode nodeEnd;
  nodeEnd.x = 11;
  nodeEnd.y = 25;

  // Set Start and goal states

  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

  unsigned int SearchState;
  unsigned int SearchSteps = 0;

  do {
    SearchState = astarsearch.SearchStep();

    SearchSteps++;

    if constexpr (DEBUG_LISTS == 1) {
      std::cout << "Steps:" << SearchSteps << "\n";

      int len = 0;

      std::cout << "Open:\n";
      MapSearchNode *p = astarsearch.GetOpenListStart();
      while (p) {
        len++;
        if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0)
          ((MapSearchNode *)p)->PrintNodeInfo();
        p = astarsearch.GetOpenListNext();
      }

      std::cout << "Open list has " << len << " nodes\n";

      len = 0;

      std::cout << "Closed:\n";
      p = astarsearch.GetClosedListStart();
      while (p) {
        len++;
        if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0) p->PrintNodeInfo();
        p = astarsearch.GetClosedListNext();
      }

      std::cout << "Closed list has " << len << " nodes\n";
    }

  } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

  if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
    std::cout << "Search found goal state\n";

    MapSearchNode *node = astarsearch.GetSolutionStart();

    std::vector<std::pair<double, double>> x_y_ps;
    std::vector<std::pair<double, double>> x_y_startend;

    std::cout << "Displaying solution\n";

    while (node) {
      node->PrintNodeInfo();
      x_y_ps.push_back(std::make_pair(node->x, node->y));
      node = astarsearch.GetSolutionNext();
    }

    x_y_startend.push_back(std::make_pair(nodeStart.x, nodeStart.y));
    x_y_startend.push_back(std::make_pair(nodeEnd.x, nodeEnd.y));

    // Once you're done with the solution you can free the nodes up
    astarsearch.FreeSolutionNodes();

    Gnuplot gp;
    gp << "set terminal x11 size 1100, 1200 0\n";
    gp << "set title 'A star search'\n";
    gp << "set tic scale 0\n";
    gp << "set palette rgbformula -7,2,-7\n";  // rainbow
    gp << "set cbrange [0:9]\n";
    gp << "set cblabel 'obstacle'\n";
    gp << "unset cbtics\n";
    gp << "set xrange [0:" << MAP_WIDTH << "]\n";
    gp << "set yrange [0:" << MAP_HEIGHT << "]\n";

    std::vector<std::tuple<double, double, int>> x_y_z;
    for (int i = 0; i != MAP_WIDTH; ++i)
      for (int j = 0; j != MAP_HEIGHT; ++j)
        x_y_z.push_back({i, j, GetMap(i, j)});
    gp << "plot " << gp.file1d(x_y_z) << " with image notitle, "
       << gp.file1d(x_y_startend) << " with points pointtype 7 notitle,"
       << gp.file1d(x_y_ps) << " with points lc rgb 'black' title 'path'\n";

  } else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
    std::cout << "Search terminated. Did not find goal state\n";
  }

  // Display the number of loops the search went through
  std::cout << "SearchSteps : " << SearchSteps << "\n";

  astarsearch.EnsureMemoryFreed();

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////
