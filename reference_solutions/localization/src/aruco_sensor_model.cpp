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
    "sensors.aruco.covariance", {0.025, 0.025});
  timeout_ = node.declare_parameter<double>("sensors.aruco.measurement_timeout", 0.1);

  tags_ = {
      // side 1
    {0, TagLocation{0.6096, 0, -M_PI_2}},
    {1, TagLocation{0.6096, 0.11, -M_PI_2}},
    {2, TagLocation{0.6096, 0.22, -M_PI_2}},
    {3, TagLocation{0.6096, 0.33, -M_PI_2}},
    {4, TagLocation{0.6096, -0.11, -M_PI_2}},
    {5, TagLocation{0.6096, -0.22, -M_PI_2}},
    {6, TagLocation{0.6096, -0.33, -M_PI_2}},
    // side 2
    {7, TagLocation{0.0, -0.381, M_PI}},
    {8, TagLocation{0.11, -0.381, M_PI}},
    {9, TagLocation{0.22, -0.381, M_PI}},
    {13, TagLocation{0.33, -0.381, M_PI}},
    {14, TagLocation{0.44, -0.381, M_PI}},
    {15, TagLocation{0.55, -0.381, M_PI}},
    {16, TagLocation{-0.11, -0.381, M_PI}},
    {17, TagLocation{-0.22, -0.381, M_PI}},
    {18, TagLocation{-0.33, -0.381, M_PI}},
    {19, TagLocation{-0.44, -0.381, M_PI}},
    {20, TagLocation{-0.55, -0.381, M_PI}},
    // side 3
    {21, TagLocation{-0.6096, 0, M_PI_2}},
    {22, TagLocation{-0.6096, 0.11, M_PI_2}},
    {23, TagLocation{-0.6096, 0.22, M_PI_2}},
    {24, TagLocation{-0.6096, 0.33, M_PI_2}},
    {25, TagLocation{-0.6096, -0.11, M_PI_2}},
    {26, TagLocation{-0.6096, -0.22, M_PI_2}},
    {27, TagLocation{-0.6096, -0.33, M_PI_2}},
    // side 4
    {28, TagLocation{0.0, 0.381, 0}},
    {29, TagLocation{0.11, 0.381, 0}},
    {30, TagLocation{0.22, 0.381, 0}},
    {31, TagLocation{0.33, 0.381, 0}},
    {32, TagLocation{0.44, 0.381, 0}},
    {33, TagLocation{0.55, 0.381, 0}},
    {34, TagLocation{-0.11, 0.381, 0}},
    {35, TagLocation{-0.22, 0.381, 0}},
    {36, TagLocation{-0.33, 0.381, 0}},
    {37, TagLocation{-0.44, 0.381, 0}},
    {38, TagLocation{-0.55, 0.381, 0}},
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
  return log(sqrt(pow(2 * M_PI, 2))) +
         log(sqrt(covariance_[0])) + log(sqrt(covariance_[1]));
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
    double x_diff = map_location.x - particle.x;
    double y_diff = map_location.y - particle.y;

    // rotation matrix is transposed
    body_location.x = x_diff * cos(particle.yaw) - y_diff * sin(particle.yaw);
    body_location.y = x_diff * sin(particle.yaw) + y_diff * cos(particle.yaw);

    double expected_dist = sqrt(pow(body_location.x, 2) + pow(body_location.y, 2) + pow(0.05, 2));
    double dist = sqrt(pow(tag.pose.position.x, 2) + pow(tag.pose.position.y, 2) + pow(tag.pose.position.z, 2));

    log_prob += pow(expected_dist - dist, 2) / covariance_[0];

    double bearing = atan2(tag.pose.position.y, tag.pose.position.x);
    double expected_bearing = atan2(body_location.y, body_location.x);

    const auto angular_error = angles::shortest_angular_distance(expected_bearing, bearing);
    log_prob += pow(angular_error, 2) / covariance_[1];
  }
  return log_prob;
}

bool ArucoSensorModel::IsMeasurementAvailable(const rclcpp::Time & cur_time)
{
  if (last_msg_.header.stamp.sec == 0) {
    return false;
  }
  if(last_msg_.tags.size() == 0) {
    return false;
  }
  const auto time_since_last_msg = cur_time - rclcpp::Time(last_msg_.header.stamp);
  return time_since_last_msg.seconds() < timeout_;
}

}  // namespace localization
