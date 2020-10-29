/*
***********************************************************************
* CollisionChecking.h:
* Collision detection, using a 2dbox-shaped vessel
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _CONSTRAINTCHECKING_H_
#define _CONSTRAINTCHECKING_H_

#include <pyclustering/container/kdtree.hpp>
#include "openspacedata.h"

namespace ASV::planning {

template <std::size_t max_vertex = 50, std::size_t max_ls = 50,
          std::size_t max_box = 20>
class CollisionChecking {
 public:
  explicit CollisionChecking(const CollisionData &_CollisionData)
      : ego_length_(_CollisionData.HULL_LENGTH),
        ego_width_(_CollisionData.HULL_WIDTH),
        ego_back2cog_(_CollisionData.HULL_BACK2COG),
        ego_center_local_x_(0.5 * ego_length_ - ego_back2cog_),
        ego_center_local_y_(0.0) {}

  virtual ~CollisionChecking() = default;

  // check collision, return true if collision occurs.
  // TODO: multi-thread to finish this
  bool InCollision(
      const std::vector<std::array<double, 3>> &ego_trajectory) const {
    for (auto const &state : ego_trajectory) {
      if (InCollision(state.at(0), state.at(1), state.at(2))) return true;
    }
    return false;

  }  // InCollision

  // check collision, return true if collision occurs.
  bool InCollision(const double ego_x, const double ego_y,
                   const double ego_theta) const {
    // correct CoG trajectory to center
    auto [ego_center_global_x, ego_center_global_y] = local2global(
        this->ego_center_local_x_, this->ego_center_local_y_, ego_theta);
    ego_center_global_x += ego_x;
    ego_center_global_y += ego_y;

    // update the 2dbox for ego vessel
    ASV::common::math::Box2d ego_box_(
        ASV::common::math::Vec2d(ego_center_global_x, ego_center_global_y),
        ego_theta, this->ego_length_, this->ego_width_);
    // check vertex
    for (std::size_t i = 0; i != max_vertex; ++i) {
      if (Obstacles_Vertex_.status[i] &&
          ego_box_.IsPointIn(Obstacles_Vertex_.vertex[i])) {
        return true;
      }
    }

    // check legement
    for (std::size_t i = 0; i != max_ls; ++i) {
      if (Obstacles_LineSegment_.status[i] &&
          ego_box_.HasOverlap(Obstacles_LineSegment_.linesegment[i])) {
        return true;
      }
    }

    // check box
    for (std::size_t i = 0; i != max_box; ++i) {
      if (Obstacles_Box2d_.status[i] &&
          ego_box_.HasOverlap(Obstacles_Box2d_.box2d[i])) {
        return true;
      }
    }

    return false;
  }  // InCollision

  // find the nearest obstacle, given position (x, y)
  std::vector<std::vector<double>> FindNearstObstacle(
      const std::vector<std::array<double, 3>> &positions) const {
    // convert from std::vector to vec2d
    std::size_t total_num = positions.size();
    std::vector<ASV::common::math::Vec2d> t_positions(total_num, {0, 0});

    for (std::size_t index = 0; index != total_num; index++) {
      t_positions[index].set_x(positions[index][0]);
      t_positions[index].set_y(positions[index][1]);
    }

    return FindNearstObstacle(t_positions);
  }  // FindNearstObstacle

  // find the nearest obstacle, given position (x, y)
  std::vector<std::vector<double>> FindNearstObstacle(
      const std::vector<ASV::common::math::Vec2d> &positions) const {
    std::size_t total_num = positions.size();
    std::vector<std::vector<double>> nearest_obstacles(total_num, {0});

    // kd search
    const double radius_search = 10.0;  // not used
    for (std::size_t index = 0; index != total_num; index++) {
      pyclustering::container::kdtree_searcher searcher(
          {positions[index].x(), positions[index].y()}, tree_.get_root(),
          radius_search);
      // FindNearestNode
      pyclustering::container::kdnode::ptr nearest_node =
          searcher.find_nearest_node();
      nearest_obstacles[index] = nearest_node->get_data();
    }

    return nearest_obstacles;
  }  // FindNearstObstacle

  auto Obstacles_Vertex() const noexcept { return Obstacles_Vertex_; }
  auto Obstacles_LineSegment() const noexcept { return Obstacles_LineSegment_; }
  auto Obstacles_Box2d() const noexcept { return Obstacles_Box2d_; }

  CollisionChecking &set_all_obstacls(
      const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex,
      const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment,
      const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
    set_Obstacles_Vertex(Obstacles_Vertex);
    set_Obstacles_LineSegment(Obstacles_LineSegment);
    set_Obstacles_Box2d(Obstacles_Box2d);
    updateAllCenters();
    return *this;
  }  // set_all_obstacls

 private:
  const double ego_length_;
  const double ego_width_;
  const double ego_back2cog_;
  const double ego_center_local_x_;
  const double ego_center_local_y_;

  Obstacle_Vertex<max_vertex> Obstacles_Vertex_;
  Obstacle_LineSegment<max_ls> Obstacles_LineSegment_;
  Obstacle_Box2d<max_box> Obstacles_Box2d_;

  std::vector<std::vector<double>> allcenters_;
  pyclustering::container::kdtree tree_;

  std::tuple<double, double> local2global(const double local_x,
                                          const double local_y,
                                          const double theta) const {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * local_x - svalue * local_y,
            svalue * local_x + cvalue * local_y};
  }  // local2global

  std::tuple<double, double> global2local(const double global_x,
                                          const double global_y,
                                          const double theta) const {
    double cvalue = std::cos(theta);
    double svalue = std::sin(theta);
    return {cvalue * global_x + svalue * global_y,
            cvalue * global_y - svalue * global_x};
  }  // global2local

  void set_Obstacles_Vertex(
      const std::vector<Obstacle_Vertex_Config> &Obstacles_Vertex) {
    std::size_t num_vertex = Obstacles_Vertex.size();
    if (num_vertex > max_vertex) {
      // TODO:: warning
      for (std::size_t i = 0; i != max_vertex; ++i) {
        Obstacles_Vertex_.status[i] = true;
        Obstacles_Vertex_.vertex[i] = ASV::common::math::Vec2d(
            Obstacles_Vertex[i].x, Obstacles_Vertex[i].y);
      }

    } else {
      std::fill(Obstacles_Vertex_.status.begin(),
                Obstacles_Vertex_.status.end(), false);
      for (std::size_t i = 0; i != num_vertex; ++i) {
        Obstacles_Vertex_.status[i] = true;
        Obstacles_Vertex_.vertex[i] = ASV::common::math::Vec2d(
            Obstacles_Vertex[i].x, Obstacles_Vertex[i].y);
      }
    }
  }  // set_Obstacles_Vertex

  void set_Obstacles_LineSegment(
      const std::vector<Obstacle_LineSegment_Config> &Obstacles_LineSegment) {
    std::size_t num_ls = Obstacles_LineSegment.size();
    if (num_ls > max_ls) {
      // TODO:: warning
      for (std::size_t i = 0; i != max_ls; ++i) {
        Obstacles_LineSegment_.status[i] = true;
        Obstacles_LineSegment_.linesegment[i] =
            ASV::common::math::LineSegment2d(
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].start_x,
                                         Obstacles_LineSegment[i].start_y),
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].end_x,
                                         Obstacles_LineSegment[i].end_y));
      }

    } else {
      std::fill(Obstacles_LineSegment_.status.begin(),
                Obstacles_LineSegment_.status.end(), false);
      for (std::size_t i = 0; i != num_ls; ++i) {
        Obstacles_LineSegment_.status[i] = true;
        Obstacles_LineSegment_.linesegment[i] =
            ASV::common::math::LineSegment2d(
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].start_x,
                                         Obstacles_LineSegment[i].start_y),
                ASV::common::math::Vec2d(Obstacles_LineSegment[i].end_x,
                                         Obstacles_LineSegment[i].end_y));
      }
    }

  }  // set_Obstacles_LineSegment

  void set_Obstacles_Box2d(
      const std::vector<Obstacle_Box2d_Config> &Obstacles_Box2d) {
    std::size_t num_box = Obstacles_Box2d.size();
    if (num_box > max_box) {
      // TODO:: warning
      for (std::size_t i = 0; i != max_box; ++i) {
        auto box_para = Obstacles_Box2d[i];
        Obstacles_Box2d_.status[i] = true;
        Obstacles_Box2d_.box2d[i] = ASV::common::math::Box2d(
            ASV::common::math::Vec2d(box_para.center_x, box_para.center_y),
            box_para.heading, box_para.length, box_para.width);
      }

    } else {
      std::fill(Obstacles_Box2d_.status.begin(), Obstacles_Box2d_.status.end(),
                false);
      for (std::size_t i = 0; i != num_box; ++i) {
        auto box_para = Obstacles_Box2d[i];
        Obstacles_Box2d_.status[i] = true;
        Obstacles_Box2d_.box2d[i] = ASV::common::math::Box2d(
            ASV::common::math::Vec2d(box_para.center_x, box_para.center_y),
            box_para.heading, box_para.length, box_para.width);
      }
    }

  }  // set_Obstacles_Box2d

  void updateAllCenters() {
    allcenters_.clear();
    // vertex
    for (std::size_t i = 0; i != max_vertex; ++i) {
      if (Obstacles_Vertex_.status[i]) {
        allcenters_.push_back(
            {Obstacles_Vertex_.vertex[i].x(), Obstacles_Vertex_.vertex[i].y()});
      }
    }

    //  line segment (we use 3 points: two vertex and center)
    for (std::size_t i = 0; i != max_ls; ++i) {
      if (Obstacles_LineSegment_.status[i]) {
        auto line_segment = Obstacles_LineSegment_.linesegment[i];
        allcenters_.push_back(
            {line_segment.center().x(), line_segment.center().y()});
        allcenters_.push_back(
            {line_segment.start().x(), line_segment.start().y()});
        allcenters_.push_back({line_segment.end().x(), line_segment.end().y()});
      }
    }

    // box (we use 4 corners of box)
    for (std::size_t i = 0; i != max_box; ++i) {
      if (Obstacles_Box2d_.status[i]) {
        auto corners = Obstacles_Box2d_.box2d[i].GetAllCorners();
        for (const auto &corner : corners)
          allcenters_.push_back({corner.x(), corner.y()});
      }
    }

    // update the kdtree
    tree_ = pyclustering::container::kdtree();
    for (auto &center : allcenters_) {
      tree_.insert(center);
    }

  }  // updateAllCenters

};  // end class CollisionChecking

using CollisionChecking_Astar = CollisionChecking<50, 50, 20>;

}  // namespace ASV::planning

#endif /* _CONSTRAINTCHECKING_H_ */