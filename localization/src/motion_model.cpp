//
// Created by jason on 7/5/21.
//

#include "motion_model.h"

namespace localization {


IMUMotionModel::IMUMotionModel(std::shared_ptr<ParticleNoise> noise, rclcpp::Node* node)
{
  noise_ = noise;

  node->declare_parameter<std::vector<double>>("motion_sigmas", {0.05, 0.05, 0.2, 0.05, 0.0});
  std::vector<double> motion_sigma;
  node->get_parameter("motion_sigmas", motion_sigma);
  sigmas_.x = motion_sigma[0];
  sigmas_.y = motion_sigma[1];
  sigmas_.yaw = motion_sigma[2];
  sigmas_.vx = motion_sigma[3];
  sigmas_.vy = motion_sigma[4];
}

void IMUMotionModel::updateParticle(Particle & particle, double dt,
                                 sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  particle.x += cos(particle.yaw)*particle.vx*dt + sin(particle.yaw)*particle.vy*dt + sigmas_.x*noise_->sampleGaussian()*sqrt(dt);
  particle.y += -sin(particle.yaw)*particle.vx*dt + cos(particle.yaw)*particle.vy*dt + sigmas_.y*noise_->sampleGaussian()*sqrt(dt);

  particle.vx += imu_msg->linear_acceleration.x*dt + sigmas_.vx*noise_->sampleGaussian()*sqrt(dt);
  particle.vy += imu_msg->linear_acceleration.y*dt + sigmas_.vy*noise_->sampleGaussian()*sqrt(dt);

  particle.yaw -= imu_msg->angular_velocity.z*dt + sigmas_.yaw*noise_->sampleGaussian()*sqrt(dt);
  while(particle.yaw > M_PI)
  {
    particle.yaw -= 2*M_PI;
  }
  while(particle.yaw <= -M_PI)
  {
    particle.yaw += 2*M_PI;
  }
}

void IMUMotionModel::updateParticles(std::vector<Particle> & particles,
                     sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  //std::cout << "msg: " <<
  double dt = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec*1e-9 - last_message_time_.seconds();
  if(dt > 1.0)
  {
    last_message_time_ = imu_msg->header.stamp;
    return;
  }
  //std::cout << "dt: " << dt << std::endl;
  for(Particle & particle : particles)
  {
    updateParticle(particle, dt, imu_msg);
  }
  last_message_time_ = imu_msg->header.stamp;
}
}