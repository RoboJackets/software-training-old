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

#include "motion_model.hpp"
#include <angles/angles.h>
#include <memory>
#include <vector>

namespace localization
{


MotionModel::MotionModel(rclcpp::Node * node)
{
  std::vector<double> motion_sigma = node->declare_parameter<std::vector<double>>(
    "motion_sigmas",
    {0.05, 0.05, 0.2,
      0.05, 0.0});
  sigmas_.x = motion_sigma[0];
  sigmas_.y = motion_sigma[1];
  sigmas_.yaw = motion_sigma[2];
  sigmas_.x_vel = motion_sigma[3];
  sigmas_.yaw_vel = motion_sigma[4];
}

void MotionModel::updateParticle(
  Particle & particle, double dt,
  geometry_msgs::msg::Twist::SharedPtr cmd_msg)
{
  // BEGIN STUDENT CODE
  particle.x += cos(particle.yaw) * particle.x_vel * dt + sigmas_.x * gaussian_noise_.Sample() *
    sqrt(
    dt);
  particle.y += -sin(particle.yaw) * particle.x_vel * dt + sigmas_.y * gaussian_noise_.Sample() *
    sqrt(
    dt);
  particle.yaw += particle.yaw_vel * dt + sigmas_.yaw * gaussian_noise_.Sample() * sqrt(dt);

  particle.x_vel = cmd_msg->linear.x + sigmas_.x_vel * gaussian_noise_.Sample() * sqrt(dt);
  particle.yaw_vel = -cmd_msg->angular.z + sigmas_.yaw_vel * gaussian_noise_.Sample() * sqrt(dt);

  particle.yaw = angles::normalize_angle(particle.yaw);
  // END STUDENT CODE
}

void MotionModel::updateParticles(
  std::vector<Particle> & particles,
  geometry_msgs::msg::Twist::SharedPtr cmd_msg,
  rclcpp::Time current_time)
{
  double dt = current_time.seconds() - last_message_time_.seconds();
  if (dt > 1.0) {
    last_message_time_ = current_time;
    return;
  }
  for (Particle & particle : particles) {
    updateParticle(particle, dt, cmd_msg);
  }
  last_message_time_ = current_time;
}
}  // namespace localization
