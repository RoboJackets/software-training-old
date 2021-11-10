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
#ifndef ASTAR_PATH_PLANNER_HPP_
#define ASTAR_PATH_PLANNER_HPP_

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <vector>
#include "utils.hpp"

namespace astar_path_planner
{

class AStarPathPlanner
{
public:
  using FrontierQueue = std::priority_queue<FrontierEntry, std::vector<FrontierEntry>,
      FrontierEntryComparator>;
  using ExpandedSet = std::unordered_set<Point, PointHash, PointEqualityComparator>;

  static void DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  AStarPathPlanner(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap);

  std::vector<Point> Plan(const Point & start, const Point & goal);

  const ExpandedSet & GetExpandedSet() const
  {
    return expanded_;
  }

private:
  rclcpp::Logger logger_;
  Point goal_;
  double goal_threshold_;
  double grid_size_;
  double collision_radius_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap_;
  ExpandedSet expanded_;
  FrontierQueue frontier_;

  bool IsGoal(const Point & point);

  std::vector<Point> GetAdjacentPoints(const Point & point);

  double GetHeuristicCost(const Point & point);

  double GetStepCost(const Point & point, const Point & next);

  bool PointInCollision(const Point & point);
};

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER_HPP_
