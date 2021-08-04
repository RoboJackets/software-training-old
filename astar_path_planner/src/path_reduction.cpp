#include "astar_path_planner/path_reduction.hpp"

namespace astar_path_planner
{

void ReducePath(std::vector<Point> & path)
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

bool PointsAreCollinear(std::vector<Point>::iterator first, std::vector<Point>::iterator last)
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

}