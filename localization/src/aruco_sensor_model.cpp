//
// Created by jason on 7/9/21.
//

#include "aruco_sensor_model.h"

namespace localization
{

ArucoSensorModel::ArucoSensorModel(rclcpp::Node* node)
{
  node->declare_parameter<std::vector<double>>("aruco_meas_cov", {0.5, 0.5});
  node->get_parameter("aruco_meas_cov", this->meas_cov_);
}

void ArucoSensorModel::UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr & tags)
{
  last_msg_ = tags;
}

double ArucoSensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2*M_PI, 2))) +
    log(sqrt(meas_cov_[0])) + log(sqrt(meas_cov_[1]));
}

double ArucoSensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  for(stsl_interfaces::msg::Tag & tag : last_msg_->tags)
  {
    // TODO convert tag from global to body frame
    TagLocation body_location;
    body_location.x = tag.pose.position.x*cos(particle.yaw) + tag.pose.position.y*sin(particle.yaw);
    body_location.y = -tag.pose.position.x*sin(particle.yaw) + tag.pose.position.y*cos(particle.yaw);

    log_prob += pow(body_location.x - particle.x, 2)/meas_cov_[0];
    log_prob += pow(body_location.y - particle.y, 2)/meas_cov_[1];

  }
  return log_prob;
}

bool ArucoSensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec*1e-9 < cur_time.seconds() - this->time_delay_;
}

}