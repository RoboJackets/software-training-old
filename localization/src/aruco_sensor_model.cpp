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
#include <vector>
#include <utility>

namespace localization
{

ArucoSensorModel::ArucoSensorModel(rclcpp::Node * node)
{
  // BEGIN STUDENT CODE
  this->meas_cov_ = node->declare_parameter<std::vector<double>>(
    "aruco/meas_cov", {0.025, 0.025,
      0.025});
  this->time_delay_ = node->declare_parameter<double>("aruco/time_delay", 0.1);

  tags_.insert(std::pair<int, TagLocation>(0, TagLocation(0.6096, 0, -M_PI_2)));
  tags_.insert(std::pair<int, TagLocation>(1, TagLocation(0.3, -0.381, M_PI)));
  tags_.insert(std::pair<int, TagLocation>(2, TagLocation(-0.3, -0.381, M_PI)));
  tags_.insert(std::pair<int, TagLocation>(3, TagLocation(-0.6096, 0, M_PI_2)));
  tags_.insert(std::pair<int, TagLocation>(4, TagLocation(-0.3, 0.381, 0)));
  tags_.insert(std::pair<int, TagLocation>(5, TagLocation(0.3, 0.381, 0)));

  tag_sub_ = node->create_subscription<stsl_interfaces::msg::TagArray>(
    "/tags", 1, std::bind(&ArucoSensorModel::UpdateMeasurement, this, std::placeholders::_1));

  // END STUDENT CODE
}

void ArucoSensorModel::UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr tags)
{
  last_msg_ = tags;
}

double ArucoSensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2 * M_PI, 3))) +
         log(sqrt(meas_cov_[0])) + log(sqrt(meas_cov_[1])) + log(sqrt(meas_cov_[2]));
}

double ArucoSensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  for (const stsl_interfaces::msg::Tag & tag : last_msg_->tags) {
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
    body_location.yaw = map_location.yaw + particle.yaw;
    while (body_location.yaw > M_PI) {
      body_location.yaw -= 2 * M_PI;
    }
    while (body_location.yaw < -M_PI) {
      body_location.yaw += 2 * M_PI;
    }

    log_prob += pow(body_location.x - tag.pose.position.x, 2) / meas_cov_[0];
    log_prob += pow(body_location.y - tag.pose.position.y, 2) / meas_cov_[1];

    // handles the error difference cleanly
    double yaw_error = body_location.yaw - yaw;
    // yaw error cannot exceed pi
    while (std::abs(yaw_error) > M_PI) {
      yaw_error = std::abs(yaw_error) - M_PI;
    }
    log_prob += pow(yaw_error, 2) / meas_cov_[2];
  }
  return log_prob;
}

bool ArucoSensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  if (!last_msg_) {
    return false;
  }
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec * 1e-9 <
         cur_time.seconds() - this->time_delay_;
}

}  // namespace localization
