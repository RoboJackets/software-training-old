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

#include "test_path_generator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

namespace lqr_control
{

TestPathGenerator::TestPathGenerator(const std::size_t point_count)
: point_count_(point_count),
  t_delta_((2 * M_PI) / (point_count_))
{
}

nav_msgs::msg::Path TestPathGenerator::BuildPath()
{
  nav_msgs::msg::Path path;
  t_ = 0.0;
  std::generate_n(std::back_inserter(path.poses), point_count_, [this] {return GetNextPoint();});
  return path;
}

geometry_msgs::msg::PoseStamped TestPathGenerator::GetNextPoint()
{
  /* Builds a Gerono lemniscate.
   * Math taken from https://mathworld.wolfram.com/EightCurve.html
   */

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = scale_ * std::sin(t_);
  pose.pose.position.y = scale_ * std::sin(t_) * std::cos(t_);

  const auto dx = scale_ * std::cos(t_);
  const auto dy = (scale_ * std::cos(t_) * std::cos(t_)) - (scale_ * std::sin(t_) * std::sin(t_));

  pose.pose.orientation = tf2::toMsg(
    Eigen::Quaterniond{Eigen::AngleAxisd{std::atan2(
          dy,
          dx),
        Eigen::Vector3d::UnitZ()}});

  t_ += t_delta_;
  return pose;
}

}  // namespace lqr_control
