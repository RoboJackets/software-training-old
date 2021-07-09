#include <Eigen/Dense>

namespace astar_path_planner
{

using Point = Eigen::Vector2d;

struct PointComparator
{
  bool operator()(const Point & left, const Point & right) const
  {
    return std::tie(left.x(), left.y()) < std::tie(right.x(), right.y());
  }
};

struct FrontierEntry
{
  std::vector<Point> path;
  double cost;
};

struct FrontierEntryComparator
{
  bool operator()(const FrontierEntry & a, const FrontierEntry & b) const
  {
    return a.cost > b.cost;
  }
};

}
