// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef ASTAR_PATH_PLANNER__ASTAR_PATH_PLANNER_HPP_
#define ASTAR_PATH_PLANNER__ASTAR_PATH_PLANNER_HPP_

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <memory>
#include <vector>
#include "astar_path_planner/utils.hpp"

namespace astar_path_planner
{

class AStarPathPlanner
{
public:
  static void DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  AStarPathPlanner(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap);

  std::vector<Point> Plan(const Point & start, const Point & goal);

private:
  rclcpp::Logger logger_;
  Point goal_;
  double goal_threshold_;
  double grid_size_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap_;

  bool IsGoal(const Point & point);

  std::vector<Point> GetAdjacentPoints(const Point & point);

  double GetPathCost(const std::vector<Point> & path);

  bool PointInCollision(const Point & point);

  void ReducePath(std::vector<Point> & path);
};

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER__ASTAR_PATH_PLANNER_HPP_
