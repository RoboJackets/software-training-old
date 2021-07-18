//
// Created by jason on 7/9/21.
//

#include "aruco_sensor_model.h"

namespace localization
{

ArucoSensorModel::ArucoSensorModel(rclcpp::Node* node)
{
  node->declare_parameter<std::vector<double>>("aruco/meas_cov", {0.025, 0.025, 0.025});
  node->get_parameter("aruco/meas_cov", this->meas_cov_);
  time_delay_ = 0.2;

  tags_.insert(std::pair<int, TagLocation>(0, TagLocation(0.6096, 0, -M_PI_2)));
  tags_.insert(std::pair<int, TagLocation>(1, TagLocation(0.3, -0.381, M_PI)));
  tags_.insert(std::pair<int, TagLocation>(2, TagLocation(-0.3, -0.381, M_PI)));
  tags_.insert(std::pair<int, TagLocation>(3, TagLocation(-0.6096, 0, M_PI_2)));
  tags_.insert(std::pair<int, TagLocation>(4, TagLocation(-0.3, 0.381, 0)));
  tags_.insert(std::pair<int, TagLocation>(5, TagLocation(0.3, 0.381, 0)));
}

void ArucoSensorModel::UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr & tags)
{
  last_msg_ = tags;
}

double ArucoSensorModel::ComputeLogNormalizer()
{
  return log(sqrt(pow(2*M_PI, 3))) +
    log(sqrt(meas_cov_[0])) + log(sqrt(meas_cov_[1])) + log(sqrt(meas_cov_[2]));
}

double ArucoSensorModel::ComputeLogProb(Particle & particle)
{
  double log_prob = 0;
  //std::cout << "==========" << std::endl;
  for(const stsl_interfaces::msg::Tag & tag : last_msg_->tags)
  {
    // TODO convert tag from global to body frame
    TagLocation body_location;
    if (tags_.find(tag.id) == tags_.end())
    {
      continue;
    }
    TagLocation map_location = tags_.at(tag.id);
    //std::cout << "particle " << particle.x << ", " << particle.y << ", " << particle.yaw << std::endl;
    //std::cout << "map location (" << map_location.x << ", " << map_location.y << ", " << map_location.yaw << ")" << std::endl;
    body_location.x = map_location.x*cos(particle.yaw) - map_location.y*sin(particle.yaw) - particle.x*cos(particle.yaw) + particle.y*sin(particle.yaw);
    body_location.y = map_location.x*sin(particle.yaw) + map_location.y*cos(particle.yaw) - particle.x*sin(particle.yaw) - particle.y*cos(particle.yaw);

    double r,p,yaw;
    tf2::Quaternion q;
    tf2::fromMsg(tag.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r,p,yaw);
    body_location.yaw = map_location.yaw + particle.yaw;
    while(body_location.yaw > M_PI)
    {
      body_location.yaw -= 2*M_PI;
    }
    while(body_location.yaw < -M_PI)
    {
      body_location.yaw += 2*M_PI;
    }

    //std::cout << "body location (" << body_location.x << ", " << body_location.y << ", " << body_location.yaw << ")" << std::endl;
    //std::cout << "tag location (" << tag.pose.position.x << ", " << tag.pose.position.y << ", " << yaw << ")" << std::endl;

    log_prob += pow(body_location.x - tag.pose.position.x, 2)/meas_cov_[0];
    log_prob += pow(body_location.y - tag.pose.position.y, 2)/meas_cov_[1];
    log_prob += pow(body_location.yaw - yaw, 2)/meas_cov_[1];

  }
  //std::cout << log_prob << std::endl;
  return log_prob;
}

bool ArucoSensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
{
  if (!last_msg_)
  {
    return false;
  }
  return last_msg_->header.stamp.sec + last_msg_->header.stamp.nanosec*1e-9 < cur_time.seconds() - this->time_delay_;
}

}