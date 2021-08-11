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

#include "astar_path_planner.hpp"
#include <memory>
#include <set>
#include <vector>
#include <queue>

namespace astar_path_planner
{

void AStarPathPlanner::DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  node->declare_parameter("goal_threshold", 0.05);
  node->declare_parameter("grid_size", 0.01);
}

AStarPathPlanner::AStarPathPlanner(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap)
: logger_(node->get_logger()),
  ros_costmap_(ros_costmap)
{
  goal_threshold_ = node->get_parameter("goal_threshold").as_double();
  grid_size_ = node->get_parameter("grid_size").as_double();
}

std::vector<Point> AStarPathPlanner::Plan(const Point & start, const Point & goal)
{
  std::set<Point, PointComparator> expanded{PointComparator{}};
  std::priority_queue<FrontierEntry, std::vector<FrontierEntry>,
    FrontierEntryComparator> frontier{FrontierEntryComparator{}};

  goal_ = goal;

  frontier.push({{start}, GetHeuristicCost(start)});

  while (!frontier.empty()) {
    const auto [path, cost] = frontier.top();
    frontier.pop();
    const auto last_state = path.back();

    if (expanded.find(last_state) == expanded.end()) {
      expanded.insert(last_state);

      if (IsGoal(last_state)) {
        return path;
      }

      const auto neighbors = GetAdjacentPoints(last_state);

      for (const auto & neighbor : neighbors) {
        std::vector<Point> new_path(path);
        new_path.push_back(neighbor);
        const auto new_cost = cost - GetHeuristicCost(path.back()) +
          GetStepCost(path.back(), neighbor) + GetHeuristicCost(neighbor);
        frontier.push({new_path, new_cost});
      }
    }
  }

  throw std::runtime_error("Could not find path!");
}

bool AStarPathPlanner::IsGoal(const Point & point)
{
  return (point - goal_).norm() < goal_threshold_;
}

std::vector<Point> AStarPathPlanner::GetAdjacentPoints(const Point & point)
{
  std::vector<Point> neighbors;

  for (auto dx = -grid_size_; dx <= grid_size_; dx += grid_size_) {
    for (auto dy = -grid_size_; dy <= grid_size_; dy += grid_size_) {
      if (std::abs(dx) < 1e-4 && std::abs(dy) < 1e-4) {
        continue;
      }
      const Point neighbor{point.x() + dx, point.y() + dy};
      if (!PointInCollision(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}


double AStarPathPlanner::GetHeuristicCost(const Point & point)
{
  return (point - goal_).norm();
}

double AStarPathPlanner::GetStepCost(const Point & point, const Point & next)
{
  return (next - point).norm();
}

bool AStarPathPlanner::PointInCollision(const Point & point)
{
  const auto footprint = ros_costmap_->getRobotFootprintPolygon();

  std::vector<nav2_costmap_2d::MapLocation> footprint_in_map;

  auto transform_footprint_point = [&point](
    const auto & footprint_point) {
      nav2_costmap_2d::MapLocation location;
      location.x = footprint_point.x + point.x();
      location.y = footprint_point.y + point.y();
      return location;
    };

  std::transform(
    footprint.points.begin(), footprint.points.end(), std::back_inserter(
      footprint_in_map), transform_footprint_point);

  auto costmap = ros_costmap_->getCostmap();

  std::vector<nav2_costmap_2d::MapLocation> map_cells;
  costmap->convexFillCells(footprint_in_map, map_cells);

  return std::any_of(
    map_cells.begin(), map_cells.end(), [costmap](const auto & cell) {
      const auto cost = costmap->getCost(cell.x, cell.y);
      return cost == 253;
    });
}

}  // namespace astar_path_planner
