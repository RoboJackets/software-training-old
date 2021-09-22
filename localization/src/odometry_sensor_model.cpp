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

#include "odometry_sensor_model.hpp"
#include <vector>

namespace localization
{

OdometrySensorModel::OdometrySensorModel(rclcpp::Node * node)
{
  // BEGIN STUDENT CODE
  this->meas_cov_ = node->declare_parameter<std::vector<double>>("odom/meas_cov", {0.1, 0.1});
  this->time_delay_ = node->declare_parameter<double>("odom/time_delay", 0.1);

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&OdometrySensorModel::UpdateMeasurement, this, std::placeholders::_1));
  // END STUDENT CODE
}

void OdometrySensorModel::UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  last_msg_ = odom;
}

double OdometrySensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2 * M_PI, 2))) +
         log(sqrt(meas_cov_[0])) + log(sqrt(meas_cov_[1]));
}

double OdometrySensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  log_prob += pow(last_msg_->twist.twist.linear.x - particle.vx, 2) / meas_cov_[0];
  log_prob += pow(last_msg_->twist.twist.angular.z - particle.yaw_rate, 2) / meas_cov_[1];
  return log_prob;
}

bool OdometrySensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  if (!last_msg_) {
    return false;
  }
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec * 1e-9 >
         cur_time.seconds() - this->time_delay_;
}

}  // namespace localization
