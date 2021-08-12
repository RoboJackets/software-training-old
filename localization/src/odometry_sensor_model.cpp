//
// Created by jason on 7/9/21.
//

#include "odometry_sensor_model.h"

namespace localization
{

OdometrySensorModel::OdometrySensorModel(rclcpp::Node* node)
{
  this->meas_cov_ = node->declare_parameter<std::vector<double>>("odom/meas_cov", {0.1, 0.1});
  this->time_delay_ = node->declare_parameter<double>("odom/time_delay", 0.1);

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10,
          std::bind(&OdometrySensorModel::UpdateMeasurement, this, std::placeholders::_1));
}

void OdometrySensorModel::UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  last_msg_ = odom;
}

double OdometrySensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2*M_PI, 2))) +
         log(sqrt(meas_cov_[0])) + log(sqrt(meas_cov_[1]));
}

double OdometrySensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  log_prob += pow(last_msg_->twist.twist.linear.x - particle.vx, 2)/meas_cov_[0];
  log_prob += pow(last_msg_->twist.twist.angular.z - particle.yaw_rate, 2)/meas_cov_[1];
  return log_prob;
}

bool OdometrySensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  if (!last_msg_)
  {
    return false;
  }
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec*1e-9 < cur_time.seconds() - this->time_delay_;
}

}
