/*
*******************************************************************************
* HybridAStar.h:
* Hybrid A star algorithm, to generate the coarse collision-free trajectory
* This header file can be read by C++ compilers
*
* by Hu.ZH, Bin Li(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _HYBRIDASTAR_H_
#define _HYBRIDASTAR_H_

#include "CollisionChecking.h"
#include "common/math/Geometry/include/Reeds_Shepp.h"
#include "hybridstlastar.h"
#include "openspacedata.h"

#include <iostream>

namespace ASV::planning {

// 2d node for search a collison-free path
class HybridState2DNode {
  using HybridAStar_Search = HybridAStarSearch<HybridState2DNode, SearchConfig,
                                               CollisionChecking_Astar>;

 public:
  HybridState2DNode() : x_(0), y_(0), theta_(0) {}

  HybridState2DNode(float px, float py, float ptheta)
      : x_(px), y_(py), theta_(ptheta) {}

  float x() const noexcept { return x_; }
  float y() const noexcept { return y_; }
  float theta() const noexcept { return theta_; }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(const HybridState2DNode &nodeGoal,
                             std::nullptr_t) {
    return std::abs(x_ - nodeGoal.x()) + std::abs(y_ - nodeGoal.y());
  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState2DNode &nodeGoal) {
    return ((std::abs(x_ - nodeGoal.x()) <= 0.5) &&
            (std::abs(y_ - nodeGoal.y()) <= 0.5));
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState2DNode *parent_node,
                     const SearchConfig &search_config,
                     const CollisionChecking_Astar &collisionchecker) {
    constexpr static int _DIRECTION_NUM = 8;
    constexpr static int _DIMENSION = 2;
    constexpr static int move_direction[_DIRECTION_NUM][_DIMENSION] = {
        {0, 1},    // move up
        {1, 1},    // move up-right
        {1, 0},    // move right
        {1, -1},   // move right-down
        {0, -1},   // move down
        {-1, -1},  // move down-left
        {-1, 0},   // move left
        {-1, 1},   // move left-up
    };

    (void)parent_node;

    float L = search_config.move_length;
    float max_turn = search_config.turning_angle;

    // push each possible move except allowing the search to go backwards
    for (int i = 0; i != _DIRECTION_NUM; ++i) {
      float new_x = this->x_ + L * move_direction[i][0];
      float new_y = this->y_ + L * move_direction[i][1];

      for (int j = -2; j != 3; j++) {
        float new_theta = ASV::common::math::fNormalizeheadingangle(
            this->theta_ + 0.5 * max_turn * j);

        if (!collisionchecker.InCollision(new_x, new_y, new_theta)) {
          HybridState2DNode NewNode =
              HybridState2DNode(new_x, new_y, new_theta);
          astarsearch->AddSuccessor(NewNode);
        }
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  float GetCost(const HybridState2DNode &successor,
                const SearchConfig &search_config) {
    (void)search_config;
    return std::abs(successor.x() - x_) + std::abs(successor.y() - y_) +
           std::abs(ASV::common::math::fNormalizeheadingangle(
               successor.theta() - theta_));
  }  // GetCost

  bool IsSameState(const HybridState2DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    return ((std::abs(x_ - rhs.x()) <= 0.05) &&
            (std::abs(y_ - rhs.y()) <= 0.05));

  }  // IsSameState

 private:
  // the index (x_,y_,theta_) of the node
  float x_;
  float y_;
  float theta_;

  bool Issamestate(const HybridState2DNode &lhs, const HybridState2DNode &rhs) {
    return ((std::abs(lhs.x() - rhs.x()) <= 0.05) &&
            (std::abs(lhs.y() - rhs.y()) <= 0.05));

  }  // Issamestate

};  // end class HybridState2DNode

class HybridState4DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig,
                        CollisionChecking_Astar,
                        ASV::common::math::ReedsSheppStateSpace>;

  enum MovementType {
    STRAIGHT_FORWARD = 0,
    LEFTTURN_FORWARD,
    RIGHTTURN_FORWARD,
    STRAIGHT_REVERSE,
    RIGHTTURN_REVERSE,
    LEFTTURN_REVERSE
  };

 public:
  HybridState4DNode()
      : x_(0.0), y_(0.0), theta_(0.0), type_(STRAIGHT_FORWARD) {}

  HybridState4DNode(float px, float py, float ptheta,
                    MovementType ptype = STRAIGHT_FORWARD)
      : x_(px), y_(py), theta_(ptheta), type_(ptype) {}

  float x() const noexcept { return x_; }
  float y() const noexcept { return y_; }
  float theta() const noexcept { return theta_; }
  MovementType type() const noexcept { return type_; }
  bool IsForward() const noexcept { return (type_ <= 2); }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(
      const HybridState4DNode &nodeGoal,
      const ASV::common::math::ReedsSheppStateSpace &RSCurve) {
    std::array<double, 3> _rsstart = {this->x_, this->y_, this->theta_};
    std::array<double, 3> _rsend = {nodeGoal.x(), nodeGoal.y(),
                                    nodeGoal.theta()};

    float rsdistance =
        static_cast<float>(RSCurve.rs_distance(_rsstart, _rsend));
    // float rsdistance =
    //     (std::abs(this->x_ - nodeGoal.x()) + std::abs(this->y_ -
    //     nodeGoal.y()));

    return rsdistance;

  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState4DNode &nodeGoal) {
    if ((std::abs(x_ - nodeGoal.x()) <= 0.05) &&
        (std::abs(y_ - nodeGoal.y()) <= 0.05) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             theta_ - nodeGoal.theta())) < 0.02)) {
      return true;
    }
    return false;
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState4DNode *parent_node,
                     const SearchConfig &search_config,
                     const CollisionChecking_Astar &collision_checking) {
    float L = search_config.move_length;
    float max_turn = search_config.turning_angle;
    auto move_step = update_movement_step(L, max_turn, this->theta_);

    // push each possible move except allowing the search to go backwards
    for (int i = 0; i != 6; ++i) {
      auto move_at_type_i = move_step.col(i);
      float new_x = this->x_ + move_at_type_i(0);
      float new_y = this->y_ + move_at_type_i(1);
      float new_theta = ASV::common::math::fNormalizeheadingangle(
          this->theta_ + move_at_type_i(2));
      MovementType new_type = static_cast<MovementType>(i);

      HybridState4DNode NewNode =
          HybridState4DNode(new_x, new_y, new_theta, new_type);

      if (parent_node) {
        if (!(collision_checking.InCollision(new_x, new_y, new_theta) ||
              Issamestate(*parent_node, NewNode))) {
          astarsearch->AddSuccessor(NewNode);
        }
      } else {
        if (!collision_checking.InCollision(new_x, new_y, new_theta)) {
          astarsearch->AddSuccessor(NewNode);
        }
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  float GetCost(const HybridState4DNode &successor,
                const SearchConfig &_SearchConfig) {
    int current_type = static_cast<int>(this->type_);
    int successor_type = static_cast<int>(successor.type());

    return _SearchConfig.cost_map[current_type][successor_type];

  }  // GetCost

  bool IsSameState(const HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    return ((std::abs(x_ - rhs.x()) <= 0.05) &&
            (std::abs(y_ - rhs.y()) < 0.05) &&
            (std::abs(ASV::common::math::fNormalizeheadingangle(
                 theta_ - rhs.theta())) < 0.01));

  }  // IsSameState

 private:
  float x_;  // the (x,y) positions of the node
  float y_;
  float theta_;
  MovementType type_;

  Eigen::Matrix<float, 3, 6> update_movement_step(float L, float varphi,
                                                  float theta) {
    Eigen::Matrix<float, 3, 6> movement_step =
        Eigen::Matrix<float, 3, 6>::Zero();

    float L_cos_theta = L * std::cos(theta);
    float L_sin_theta = L * std::sin(theta);
    float sin_varphi = std::sin(varphi) / varphi;
    float one_cos_varphi = (1.0 - std::cos(varphi)) / varphi;
    // S+
    movement_step(0, 0) = L_cos_theta;
    movement_step(1, 0) = L_sin_theta;
    movement_step(2, 0) = 0;
    // L+
    movement_step(0, 1) =
        L_cos_theta * sin_varphi - L_sin_theta * one_cos_varphi;
    movement_step(1, 1) =
        L_sin_theta * sin_varphi + L_cos_theta * one_cos_varphi;
    movement_step(2, 1) = varphi;
    // R+
    movement_step(0, 2) =
        L_cos_theta * sin_varphi + L_sin_theta * one_cos_varphi;
    movement_step(1, 2) =
        L_sin_theta * sin_varphi - L_cos_theta * one_cos_varphi;
    movement_step(2, 2) = -varphi;

    // reverse
    for (int i = 3; i != 6; ++i) {
      movement_step(0, i) = -movement_step(0, i - 3);
      movement_step(1, i) = -movement_step(1, i - 3);
      movement_step(2, i) = movement_step(2, i - 3);
    }

    return movement_step;
  }  // update_movement_step

  bool Issamestate(const HybridState4DNode &lhs, const HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((std::abs(lhs.x() - rhs.x()) <= 0.05) &&
        (std::abs(lhs.y() - rhs.y()) <= 0.05) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             lhs.theta() - rhs.theta())) < 0.01)) {
      return true;
    }
    return false;
  }  // Issamestate

};  // end class HybridState4DNode

class HybridAStar {
  using HybridAStar_4dNode_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig,
                        CollisionChecking_Astar,
                        ASV::common::math::ReedsSheppStateSpace>;
  using HybridAStar_2dNode_Search =
      HybridAStarSearch<HybridState2DNode, SearchConfig,
                        CollisionChecking_Astar>;

 public:
  HybridAStar(const CollisionData &collisiondata,
              const HybridAStarConfig &hybridastarconfig)
      : startpoint_({0, 0, 0}),
        endpoint_({0, 0, 0}),
        rscurve_(1.0 / collisiondata.MAX_CURVATURE),
        searchconfig_({
            0,     // move_length
            0,     // turning_angle
            {{0}}  // cost_map
        }),
        astar_4d_search_(5000),
        astar_2d_search_(3000) {
    searchconfig_ = GenerateSearchConfig(collisiondata, hybridastarconfig);
  }
  virtual ~HybridAStar() = default;

  // update the start and ending points
  HybridAStar &setup_start_end(const float start_x, const float start_y,
                               const float start_theta, const float end_x,
                               const float end_y, const float end_theta) {
    startpoint_ = {start_x, start_y, start_theta};
    endpoint_ = {end_x, end_y, end_theta};

    HybridState4DNode nodeStart(start_x, start_y, start_theta);
    HybridState4DNode nodeEnd(end_x, end_y, end_theta);
    astar_4d_search_.SetStartAndGoalStates(nodeStart, nodeEnd, rscurve_);

    return *this;
  }  // setup_start_end

  // update the start and ending points
  HybridAStar &setup_2d_start_end(const float start_x, const float start_y,
                                  const float start_theta, const float end_x,
                                  const float end_y, const float end_theta) {
    HybridState2DNode nodeStart(start_x, start_y, start_theta);
    HybridState2DNode nodeEnd(end_x, end_y, end_theta);
    astar_2d_search_.SetStartAndGoalStates(nodeStart, nodeEnd);

    return *this;
  }  // setup_2d_start_end

  void perform_4dnode_search(const CollisionChecking_Astar &collision_checker) {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;
    do {
      // perform a hybrid A* search
      SearchState = astar_4d_search_.SearchStep(searchconfig_,
                                                collision_checker, rscurve_);
      SearchSteps++;

      // get the current node
      HybridState4DNode *current_p = astar_4d_search_.GetCurrentNode();
      if (current_p) {
        std::array<double, 3> closedlist_end = {current_p->x(), current_p->y(),
                                                current_p->theta()};

        std::array<double, 3> rscurve_end = {static_cast<double>(endpoint_[0]),
                                             static_cast<double>(endpoint_[1]),
                                             static_cast<double>(endpoint_[2])};

        // try a rs curve
        auto rscurve_generated = rscurve_.rs_state(
            closedlist_end, rscurve_end, 0.5 * searchconfig_.move_length);

        // check the collision for the generated RS curve
        if (!collision_checker.InCollision(rscurve_generated)) {
          std::vector<std::tuple<double, double, double, bool>>
              closedlist_trajecotry;
          while (current_p) {
            double current_node_x = static_cast<double>(current_p->x());
            double current_node_y = static_cast<double>(current_p->y());
            double current_node_theta = static_cast<double>(current_p->theta());
            bool current_move_type = current_p->IsForward();
            closedlist_trajecotry.push_back({current_node_x, current_node_y,
                                             current_node_theta,
                                             current_move_type});
            current_p = astar_4d_search_.GetCurrentNodePrev();
            // check the forward/reverse switch
            if (current_p) {
              bool pre_move_type = current_p->IsForward();
              if (pre_move_type != current_move_type)
                closedlist_trajecotry.push_back({current_node_x, current_node_y,
                                                 current_node_theta,
                                                 pre_move_type});
            }
          }  // end while
          // reverse the trajectory
          std::reverse(closedlist_trajecotry.begin(),
                       closedlist_trajecotry.end());

          // combine two kinds of trajectory
          auto rscurve_trajectory = rscurve_.rs_trajectory(
              closedlist_end, rscurve_end, 0.5 * searchconfig_.move_length);

          closedlist_trajecotry.insert(closedlist_trajecotry.end(),
                                       rscurve_trajectory.begin(),
                                       rscurve_trajectory.end());

          hybridastar_trajecotry_ = closedlist_trajecotry;
          astar_4d_search_.CancelSearch();
          std::cout << "find a collision free RS curve!\n";
        }  // end if collision checking
      }

    } while (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SUCCEEDED) {
      HybridState4DNode *node = astar_4d_search_.GetSolutionStart();
      hybridastar_trajecotry_.clear();
      while (node) {
        hybridastar_trajecotry_.push_back(
            {node->x(), node->y(), node->theta(), true});
        node = astar_4d_search_.GetSolutionNext();
      }
      // Once you're done with the solution you can free the nodes up
      astar_4d_search_.FreeSolutionNodes();
    }

    // Display the number of loops the search went through
    std::cout << "SearchSteps : " << SearchSteps << "\n";
    // astarsearch_.FreeSolutionNodes();
    astar_4d_search_.EnsureMemoryFreed();

  }  // perform_4dnode_search

  void perform_2dnode_search(const CollisionChecking_Astar &collision_checker) {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      // perform a hybrid A* search
      SearchState =
          astar_2d_search_.SearchStep(searchconfig_, collision_checker);
      SearchSteps++;

      // get the current node
      std::vector<std::array<double, 3>> closedlist_trajecotry;
      HybridState2DNode *current_p = astar_2d_search_.GetCurrentNode();
      while (current_p) {
        closedlist_trajecotry.push_back(
            {current_p->x(), current_p->y(), current_p->theta()});
        current_p = astar_2d_search_.GetCurrentNodePrev();
      }
      std::reverse(closedlist_trajecotry.begin(), closedlist_trajecotry.end());

      // try a collision-free RS curve
      if (!closedlist_trajecotry.empty()) {
        hybridastar_2d_trajecotry_ = closedlist_trajecotry;
      }

    } while (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SUCCEEDED) {
      std::cout << "find 2d solution\n";
      HybridState2DNode *node = astar_2d_search_.GetSolutionStart();
      hybridastar_2d_trajecotry_.clear();
      while (node) {
        hybridastar_2d_trajecotry_.push_back(
            {node->x(), node->y(), node->theta()});
        node = astar_2d_search_.GetSolutionNext();
      }
      // Once you're done with the solution you can free the nodes up
      astar_2d_search_.FreeSolutionNodes();
    }

    // Display the number of loops the search went through
    std::cout << "SearchSteps : " << SearchSteps << "\n";
    // astarsearch_.FreeSolutionNodes();
    astar_2d_search_.EnsureMemoryFreed();

  }  // perform_2dnode_search

  auto hybridastar_trajecotry() const noexcept {
    return hybridastar_trajecotry_;
  }  // hybridastar_trajecotry

  auto hybridastar_2dtrajecotry() const noexcept {
    return hybridastar_2d_trajecotry_;
  }  // hybridastar_2dtrajecotry

  std::array<float, 3> startpoint() const noexcept { return startpoint_; }
  std::array<float, 3> endpoint() const noexcept { return endpoint_; }

 private:
  std::array<float, 3> startpoint_;
  std::array<float, 3> endpoint_;

  ASV::common::math::ReedsSheppStateSpace rscurve_;
  SearchConfig searchconfig_;
  HybridAStar_4dNode_Search astar_4d_search_;
  HybridAStar_2dNode_Search astar_2d_search_;

  // search results
  std::vector<std::tuple<double, double, double, bool>> hybridastar_trajecotry_;

  std::vector<std::array<double, 3>> hybridastar_2d_trajecotry_;

  // generate the config for search
  SearchConfig GenerateSearchConfig(
      const CollisionData &collisiondata,
      const HybridAStarConfig &hybridastarconfig) {
    SearchConfig searchconfig;

    float L = hybridastarconfig.move_length;
    float Ct = hybridastarconfig.penalty_turning;
    float Cr = hybridastarconfig.penalty_reverse;
    float Cs = hybridastarconfig.penalty_switch;

    searchconfig.move_length = L;
    searchconfig.turning_angle = L * collisiondata.MAX_CURVATURE;
    //
    for (int i = 0; i != 3; ++i) {
      for (int j = 0; j != 3; ++j) {
        if (i == j)
          searchconfig.cost_map[i][j] = L;
        else
          searchconfig.cost_map[i][j] = L * Ct;
      }
      for (int j = 3; j != 6; ++j)
        searchconfig.cost_map[i][j] = L * Ct * Cr * Cs;
    }
    for (int i = 3; i != 6; ++i) {
      for (int j = 0; j != 3; ++j) searchconfig.cost_map[i][j] = L * Ct * Cs;
      for (int j = 3; j != 6; ++j) {
        if (i == j)
          searchconfig.cost_map[i][j] = L * Cr;
        else
          searchconfig.cost_map[i][j] = L * Cr * Ct;
      }
    }

    return searchconfig;
  }  // GenerateSearchConfig

  // tempo function
  float twodnode_search(const CollisionChecking_Astar &collision_checker) {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;
    float path_length = 0;
    do {
      // perform a hybrid A* search
      SearchState =
          astar_2d_search_.SearchStep(searchconfig_, collision_checker);
      SearchSteps++;

    } while (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SUCCEEDED) {
      HybridState2DNode *node = astar_2d_search_.GetSolutionStart();
      float previous_x = node->x();
      float previous_y = node->y();

      while (node) {
        path_length +=
            std::hypot(previous_x - node->x(), previous_y - node->y());
        previous_x = node->x();
        previous_y = node->y();

        node = astar_2d_search_.GetSolutionNext();
      }
      // Once you're done with the solution you can free the nodes up
      astar_2d_search_.FreeSolutionNodes();
    }
    return path_length;

  }  // twodnode_search

};  // end class HybridAStar

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */