//
// Created by jason on 7/5/21.
//

#include "motion_model.h"

namespace localization {


IMUMotionModel::IMUMotionModel(std::shared_ptr<ParticleNoise> noise, rclcpp::Node* node)
{
  noise_ = noise;

  std::vector<double> motion_sigma = node->declare_parameter<std::vector<double>>("motion_sigmas", {0.05, 0.05, 0.2, 0.05, 0.0});
  sigmas_.x = motion_sigma[0];
  sigmas_.y = motion_sigma[1];
  sigmas_.yaw = motion_sigma[2];
  sigmas_.vx = motion_sigma[3];
  sigmas_.yaw_rate = motion_sigma[4];
}

void IMUMotionModel::updateParticle(Particle & particle, double dt,
                                 geometry_msgs::msg::Twist::SharedPtr cmd_msg)
{
  // BEGIN STUDENT CODE
  particle.x += cos(particle.yaw)*particle.vx*dt + sigmas_.x*noise_->sampleGaussian()*sqrt(dt);
  particle.y += -sin(particle.yaw)*particle.vx*dt + sigmas_.y*noise_->sampleGaussian()*sqrt(dt);
  particle.yaw += particle.yaw_rate*dt + sigmas_.yaw*noise_->sampleGaussian()*sqrt(dt);

  particle.vx = cmd_msg->linear.x + sigmas_.vx*noise_->sampleGaussian()*sqrt(dt);
  particle.yaw_rate = -cmd_msg->angular.z + sigmas_.yaw_rate*noise_->sampleGaussian()*sqrt(dt);

  while(particle.yaw > M_PI)
  {
    particle.yaw -= 2*M_PI;
  }
  while(particle.yaw <= -M_PI)
  {
    particle.yaw += 2*M_PI;
  }
  // END STUDENT CODE
}

void IMUMotionModel::updateParticles(std::vector<Particle> & particles,
                     geometry_msgs::msg::Twist::SharedPtr cmd_msg,
                     rclcpp::Time current_time)
{
  double dt = current_time.seconds() - last_message_time_.seconds();
  if(dt > 1.0)
  {
    last_message_time_ = current_time;
    return;
  }
  for(Particle & particle : particles)
  {
    updateParticle(particle, dt, cmd_msg);
  }
  last_message_time_ = current_time;
}
}
