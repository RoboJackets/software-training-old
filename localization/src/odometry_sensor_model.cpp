//
// Created by jason on 7/9/21.
//

#include "odometry_sensor_model.h"

namespace localization
{

OdometrySensorModel::OdometrySensorModel(rclcpp::Node* node)
{
  node->declare_parameter<std::vector<double>>("odom_meas_cov", {0.1});
  node->get_parameter("odom_meas_cov", this->meas_cov_);
}

void OdometrySensorModel::UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr & odom)
{
  last_msg_ = odom;
}

double OdometrySensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2*M_PI, 1))) +
         log(sqrt(meas_cov_[0]));
}

double OdometrySensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  log_prob += pow(last_msg_->twist.twist.linear.x - particle.vx, 2)/meas_cov_[0];
  // vy should be zero since we are not drifting these
  return log_prob;
}

bool OdometrySensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec*1e-9 < cur_time.seconds() - this->time_delay_;
}

}