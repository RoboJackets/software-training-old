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

#ifndef TEST_PATH_GENERATOR_HPP_
#define TEST_PATH_GENERATOR_HPP_

#include <nav_msgs/msg/path.hpp>

namespace controllers
{

class TestPathGenerator
{
public:
  explicit TestPathGenerator(const std::size_t point_count);

  nav_msgs::msg::Path BuildPath();

private:
  const double scale_ = 0.5;
  const std::size_t point_count_;
  double t_delta_;
  double t_ = 0.0;

  geometry_msgs::msg::PoseStamped GetNextPoint();
};

}  // namespace controllers

#endif  // TEST_PATH_GENERATOR_HPP_
