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
#include "path_reduction.hpp"
#include <vector>

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

}  // namespace astar_path_planner
