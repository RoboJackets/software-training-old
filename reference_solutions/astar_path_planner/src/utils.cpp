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

#include "utils.hpp"
#include <algorithm>
#include <vector>

namespace astar_path_planner
{

bool PointEqualityComparator::operator()(const Point & left, const Point & right) const
{
  return left.isApprox(right);
}

std::size_t PointHash::operator()(const Point & point) const
{
  std::size_t hash = 0;
  hash ^= std::hash<int>{}(std::round(point.x() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  hash ^= std::hash<int>{}(std::round(point.y() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  return hash;
}

bool FrontierEntryComparator::operator()(const FrontierEntry & a, const FrontierEntry & b) const
{
  return a.cost > b.cost;
}

std::vector<nav2_costmap_2d::MapLocation> PolygonForCircle(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & center,
  const double & radius,
  const std::size_t & resolution)
{
  std::vector<nav2_costmap_2d::MapLocation> polygon;

  auto generate_point = [&, angle = 0.0]() mutable {
      const double wx = (radius * std::cos(angle)) + center.x();
      const double wy = (radius * std::sin(angle)) + center.y();
      angle += (2 * M_PI) / resolution;
      int mx = 0;
      int my = 0;
      costmap.worldToMapNoBounds(wx, wy, mx, my);
      return nav2_costmap_2d::MapLocation{static_cast<unsigned int>(mx),
      static_cast<unsigned int>(my)};
    };

  std::generate_n(std::back_inserter(polygon), resolution, generate_point);

  return polygon;
}

}  // namespace astar_path_planner
