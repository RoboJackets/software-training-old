#include "astar_path_planner/astar_path_planner.hpp"
#include <set>
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

  std::vector<Point> initial_path = {start};
  frontier.push({initial_path, GetPathCost(initial_path)});

  while (!frontier.empty()) {
    const auto [path, cost] = frontier.top();
    frontier.pop();
    const auto last_state = path.back();

    if (expanded.find(last_state) == expanded.end()) {
      expanded.insert(last_state);

      if (IsGoal(last_state)) {
        std::vector<Point> reduced_path{path};
        ReducePath(reduced_path);
        return reduced_path;
      }

      const auto neighbors = GetAdjacentPoints(last_state);

      for (const auto & neighbor : neighbors) {
        std::vector<Point> new_path(path);
        new_path.push_back(neighbor);
        const auto cost = GetPathCost(new_path);
        frontier.push({new_path, cost});
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

double AStarPathPlanner::GetPathCost(const std::vector<Point> & path)
{
  // Distance between points
  std::vector<Eigen::Vector2d> edges;
  std::adjacent_difference(path.begin(), path.end(), std::back_inserter(edges));
  double cost = std::accumulate(
    edges.begin(), edges.end(), 0.0, [](const double & total, const Eigen::Vector2d & edge) {
      return total + edge.norm();
    });

  // Heuristic
  cost += (path.back() - goal_).norm();

  return cost;
}

bool AStarPathPlanner::PointInCollision(const Point & point)
{
  const auto footprint = ros_costmap_->getRobotFootprintPolygon();

  std::vector<nav2_costmap_2d::MapLocation> footprint_in_map;
  std::transform(
    footprint.points.begin(), footprint.points.end(), std::back_inserter(footprint_in_map), [&point](
      const auto & footprint_point) {
      nav2_costmap_2d::MapLocation location;
      location.x = footprint_point.x + point.x();
      location.y = footprint_point.y + point.y();
      return location;
    });

  auto costmap = ros_costmap_->getCostmap();

  std::vector<nav2_costmap_2d::MapLocation> map_cells;
  costmap->convexFillCells(footprint_in_map, map_cells);

  return std::any_of(
    map_cells.begin(), map_cells.end(), [costmap](const auto & cell) {
      const auto cost = costmap->getCost(cell.x, cell.y);
      return cost == 253;
    });
}

template<typename InputIterator>
bool PointsAreCollinear(InputIterator first, InputIterator last)
{
  const Eigen::Vector2d first_point = *first;
  const Eigen::Vector2d line_vector = *last - first_point;
  auto point_is_collinear = [&first_point, &line_vector](const Eigen::Vector2d & point) {
      const Eigen::Vector2d point_vector = point - first_point;
      const auto cos_of_angle = point_vector.dot(line_vector) /
        (point_vector.norm() * line_vector.norm());
      return std::abs(cos_of_angle - 1.0) < 1e-3;
    };
  return std::all_of(first, last, point_is_collinear);
}

void AStarPathPlanner::ReducePath(std::vector<Point> & path)
{
  auto first = path.begin();
  auto last = first + 2;

  while (true) {
    if (last == path.end()) {
      path.erase(first + 1, last - 1);
      return;
    } else if (PointsAreCollinear(first, last)) {
      last++;
    } else {
      first = path.erase(first + 1, last) - 1;
      last = first + 2;
    }
  }
}

}
