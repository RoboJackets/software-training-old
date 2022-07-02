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
  node->declare_parameter("goal_threshold", 0.015);
  node->declare_parameter("grid_size", 0.01);
  node->declare_parameter("collision_radius", 0.08);
}

AStarPathPlanner::AStarPathPlanner(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap)
: logger_(node->get_logger()),
  ros_costmap_(ros_costmap)
{
  goal_threshold_ = node->get_parameter("goal_threshold").as_double();
  grid_size_ = node->get_parameter("grid_size").as_double();
  collision_radius_ = node->get_parameter("collision_radius").as_double();
}

std::vector<Point> AStarPathPlanner::Plan(const Point & start, const Point & goal)
{
  if (IsPointInCollision(goal)) {
    RCLCPP_ERROR(logger_, "Provided goal position would cause a collision");
    return {};
  }

  if (IsPointInCollision(start)) {
    RCLCPP_ERROR(
      logger_, "Starting position (%f, %f) is currently in a collision",
      start.x(), start.y());
    return {};
  }

  expanded_.clear();
  frontier_ = FrontierQueue{};

  goal_ = goal;

  // BEGIN STUDENT CODE

  frontier_.push({{start}, GetHeuristicCost(start)});

  while (!frontier_.empty()) {
    // Get next state to expand
    const auto entry = frontier_.top();
    frontier_.pop();
    const auto path = entry.path;
    const auto cost = entry.cost;
    const auto last_state = path.back();

    // Skip if we've already expanded this state
    if (expanded_.count(last_state) > 0) {
      continue;
    }

    // Add state to expanded set
    expanded_.insert(last_state);

    // Check if we've found our goal
    if (IsGoal(last_state)) {
      return path;
    }

    const auto neighbors = GetAdjacentPoints(last_state);

    std::for_each(
      neighbors.begin(), neighbors.end(), [this, &path, &cost](const auto & neighbor) {
        ExtendPathAndAddToFrontier(path, cost, neighbor);
      });
  }

  RCLCPP_ERROR(logger_, "No path found after exhausting search space.");
  return {};
  // END STUDENT CODE
}

void AStarPathPlanner::ExtendPathAndAddToFrontier(
  const std::vector<Point> & path, const double & path_cost,
  const Point & next_point)
{
  // BEGIN STUDENT CODE
  std::vector<Point> new_path(path);
  new_path.push_back(next_point);
  const auto new_cost = path_cost - GetHeuristicCost(path.back()) +
    GetStepCost(path.back(), next_point) + GetHeuristicCost(next_point);
  frontier_.push({new_path, new_cost});
  // END STUDENT CODE
}

std::vector<Point> AStarPathPlanner::GetAdjacentPoints(const Point & point)
{
  // BEGIN STUDENT CODE
  std::vector<Point> neighbors;

  for (auto dx = -grid_size_; dx <= grid_size_; dx += grid_size_) {
    for (auto dy = -grid_size_; dy <= grid_size_; dy += grid_size_) {
      if (std::abs(dx) < 1e-4 && std::abs(dy) < 1e-4) {
        continue;
      }
      const Point neighbor = point + Point{dx, dy};
      if (!IsPointInCollision(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
  // END STUDENT CODE
}


double AStarPathPlanner::GetHeuristicCost(const Point & point)
{
  // BEGIN STUDENT CODE
  return (point - goal_).norm();
  // END STUDENT CODE
}

double AStarPathPlanner::GetStepCost(const Point & point, const Point & next)
{
  // BEGIN STUDENT CODE
  return (next - point).norm();
  // END STUDENT CODE
}

bool AStarPathPlanner::IsGoal(const Point & point)
{
  // BEGIN STUDENT CODE
  return (point - goal_).norm() < goal_threshold_;
  // END STUDENT CODE
}

bool AStarPathPlanner::IsPointInCollision(const Point & point)
{
  auto costmap = ros_costmap_->getCostmap();

  const auto polygon = PolygonForCircle(*costmap, point, collision_radius_);

  std::vector<nav2_costmap_2d::MapLocation> map_cells;
  costmap->convexFillCells(polygon, map_cells);

  auto is_cell_lethal = [&costmap](const auto & cell) {
      if (cell.x < 0 || cell.y < 0 || cell.x >= costmap->getSizeInCellsX() ||
        cell.y >= costmap->getSizeInCellsY())
      {
        // out of bounds cells are assumed to be empty
        return false;
      }
      const auto cost = costmap->getCost(cell.x, cell.y);
      return cost == nav2_costmap_2d::LETHAL_OBSTACLE;
    };

  return std::any_of(map_cells.begin(), map_cells.end(), is_cell_lethal);
}

}  // namespace astar_path_planner
