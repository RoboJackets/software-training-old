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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace astar_path_planner
{

using Point = Eigen::Vector2d;

struct PointEqualityComparator
{
  bool operator()(const Point & left, const Point & right) const;
};

struct PointHash
{
  std::size_t operator()(const Point & point) const;
};

struct FrontierEntry
{
  std::vector<Point> path;
  double cost;
};

struct FrontierEntryComparator
{
  bool operator()(const FrontierEntry & a, const FrontierEntry & b) const;
};

std::vector<nav2_costmap_2d::MapLocation> PolygonForCircle(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & center,
  const double & radius,
  const std::size_t & resolution = 20);

}  // namespace astar_path_planner

#endif  // UTILS_HPP_
