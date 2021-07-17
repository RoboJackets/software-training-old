//
// Created by jason on 7/5/21.
//

#include "motion_model.h"

namespace localization {


IMUMotionModel::IMUMotionModel(std::shared_ptr<ParticleNoise> noise, rclcpp::Node* node)
{
  noise_ = noise;

  node->declare_parameter<std::vector<double>>("motion_sigmas", {0.1, 0.1, 0.1, 0.2, 0.2});
  std::vector<double> motion_sigma;
  node->get_parameter("aruco_meas_cov", motion_sigma);
  sigmas_.x = motion_sigma[0];
  sigmas_.y = motion_sigma[1];
  sigmas_.yaw = motion_sigma[2];
  sigmas_.vx = motion_sigma[3];
  sigmas_.vy = motion_sigma[4];
}

void IMUMotionModel::updateParticle(Particle & particle,
                                 sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  double dt = 0.1;
  particle.x += particle.vx*dt + sigmas_.x*noise_->sampleGaussian()*sqrt(dt);
  particle.y += particle.vy*dt + sigmas_.y*noise_->sampleGaussian()*sqrt(dt);

  double ux = imu_msg->linear_acceleration.x*dt + sigmas_.vx*noise_->sampleGaussian()*sqrt(dt);
  double uy = imu_msg->linear_acceleration.y*dt + sigmas_.vy*noise_->sampleGaussian()*sqrt(dt);

  particle.vx += cos(particle.yaw)*ux + sin(particle.yaw)*uy;
  particle.vx += sin(particle.yaw)*ux + cos(particle.yaw)*uy;

  particle.yaw += imu_msg->angular_velocity.z*dt + sigmas_.yaw*noise_->sampleGaussian()*sqrt(dt);
}

void IMUMotionModel::updateParticles(std::vector<Particle> & particles,
                     sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  for(Particle & particle : particles)
  {
    updateParticle(particle, imu_msg);
  }
}
}