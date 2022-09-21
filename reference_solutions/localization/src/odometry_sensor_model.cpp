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
#include <cmath>
#include <vector>

namespace localization
{

OdometrySensorModel::OdometrySensorModel(rclcpp::Node & node)
{
  covariance_ = node.declare_parameter<std::vector<double>>("sensors.odom.covariance", {0.1, 0.1});
  timeout_ = node.declare_parameter<double>("sensors.odom.measurement_timeout", 0.1);

  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS(),
    std::bind(&OdometrySensorModel::UpdateMeasurement, this, std::placeholders::_1));
}

void OdometrySensorModel::UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_msg_ = *msg;
}

double OdometrySensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2 * M_PI, 2))) +
         log(sqrt(covariance_[0])) + log(sqrt(covariance_[1]));
}

double OdometrySensorModel::ComputeLogProb(const Particle & particle)
{
  double log_prob = 0.0;
  log_prob += pow(last_msg_.twist.twist.linear.x - particle.x_vel, 2) / covariance_[0];
  log_prob += pow(last_msg_.twist.twist.angular.z - particle.yaw_vel, 2) / covariance_[1];
  return log_prob;
}

bool OdometrySensorModel::IsMeasurementAvailable(const rclcpp::Time & current_time)
{
  if (last_msg_.header.stamp.sec == 0) {
    return false;
  }
  const auto time_since_last_msg = current_time - rclcpp::Time(last_msg_.header.stamp);
  return time_since_last_msg.seconds() < timeout_;
}

}  // namespace localization
