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

#ifndef MOTION_MODEL_HPP_
#define MOTION_MODEL_HPP_

#include <memory>
#include <random>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "particle.hpp"
#include "random_helpers.hpp"

namespace localization
{

class MotionModel
{
public:
  explicit MotionModel(rclcpp::Node & node);

  void updateParticle(Particle & particle, double dt, geometry_msgs::msg::Twist::SharedPtr cmd_msg);
  void updateParticles(
    std::vector<Particle> & particles,
    geometry_msgs::msg::Twist::SharedPtr cmd_msg,
    rclcpp::Time current_time);
  bool getEnabled(const rclcpp::Time & time);

private:
  Particle sigmas_;
  rclcpp::Time last_message_time_;
  GaussianRandomGenerator gaussian_noise_;
};

}  // namespace localization


#endif  // MOTION_MODEL_HPP_
