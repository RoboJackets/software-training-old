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

#include "aruco_sensor_model.hpp"
#include <angles/angles.h>
#include <vector>
#include <utility>

namespace localization
{

ArucoSensorModel::ArucoSensorModel(rclcpp::Node & node)
{
  covariance_ = node.declare_parameter<std::vector<double>>(
    "sensors/aruco/covariance", {0.025, 0.025,
      0.025});
  timeout_ = node.declare_parameter<double>("sensors/aruco/measurement_timeout", 0.1);

  tags_ = {
    {0, TagLocation{0.6096, 0, -M_PI_2}},
    {1, TagLocation{0.3, -0.381, M_PI}},
    {2, TagLocation{-0.3, -0.381, M_PI}},
    {3, TagLocation{-0.6096, 0, M_PI_2}},
    {4, TagLocation{-0.3, 0.381, 0}},
    {5, TagLocation{0.3, 0.381, 0}}
  };

  tag_sub_ = node.create_subscription<stsl_interfaces::msg::TagArray>(
    "/tags", 1, std::bind(&ArucoSensorModel::UpdateMeasurement, this, std::placeholders::_1));
}

void ArucoSensorModel::UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr msg)
{
  last_msg_ = *msg;
}

double ArucoSensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2 * M_PI, 3))) +
         log(sqrt(covariance_[0])) + log(sqrt(covariance_[1])) + log(sqrt(covariance_[2]));
}

double ArucoSensorModel::ComputeLogProb(const Particle & particle)
{
  double log_prob = 0;
  for (const stsl_interfaces::msg::Tag & tag : last_msg_.tags) {
    TagLocation body_location;
    // ensure this is a localization tag
    if (tags_.find(tag.id) == tags_.end()) {
      continue;
    }
    // convert the global location of the current tag id into body frame
    TagLocation map_location = tags_.at(tag.id);
    body_location.x = map_location.x * cos(particle.yaw) - map_location.y * sin(particle.yaw) -
      particle.x * cos(particle.yaw) + particle.y * sin(particle.yaw);
    body_location.y = map_location.x * sin(particle.yaw) + map_location.y * cos(particle.yaw) -
      particle.x * sin(particle.yaw) - particle.y * cos(particle.yaw);

    double r, p, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(tag.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    body_location.yaw = angles::normalize_angle(map_location.yaw + particle.yaw);

    log_prob += pow(body_location.x - tag.pose.position.x, 2) / covariance_[0];
    log_prob += pow(body_location.y - tag.pose.position.y, 2) / covariance_[1];

    const auto yaw_error = angles::shortest_angular_distance(yaw, body_location.yaw);
    log_prob += pow(yaw_error, 2) / covariance_[2];
  }
  return log_prob;
}

bool ArucoSensorModel::IsMeasurementAvailable(const rclcpp::Time & cur_time)
{
  if (last_msg_.header.stamp.sec == 0) {
    return false;
  }
  const auto time_since_last_msg = cur_time - rclcpp::Time(last_msg_.header.stamp);
  return time_since_last_msg.seconds() < timeout_;
}

}  // namespace localization
