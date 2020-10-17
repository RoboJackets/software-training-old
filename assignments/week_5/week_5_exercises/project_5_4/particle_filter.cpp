//
// Created by jason on 10/17/20.
//

#include "particle_filter.h"

void ParticleFilter::initParticles() {
  // TODO have multiple based off of swtich

  pNh.param("num_particles", num_particles_, 100);


  int init_type;
  pNh.param("init_type", init_type, 0);
  double x_mean, x_var, y_mean, y_var, yaw_mean, yaw_var;
  pNh.param("x_mean", x_mean, 0.0);
  pNh.param("x_var", x_var, 1.0);
  pNh.param("y_mean", y_mean, 0.0);
  pNh.param("y_var", y_var, 0.0);
  pNh.param("yaw_mean", yaw_mean, 0.0);
  pNh.param("yaw_var", yaw_var, 0.0);

  switch (init_type) {
    case 0:
      // TODO standard init
      for(int i = 0; i < num_particles_; i++) {
        Particle particle;
        particle.x = x_mean + distribution_(generator_)*x_var;
        particle.y = y_mean + distribution_(generator_)*y_var;
        particle.yaw = yaw_mean + distribution_(generator_)*yaw_var;
      }
      break;
    case 1:
      // TODO all facing one direction
      break;
    case 2:
      // TODO
      break;
  }
}


void ParticleFilter::gpsCallback(geometry_msgs::PointStampedConstPtr& msg) {

}

void ParticleFilter::gpsSensorUpdate(geometry_msgs::PointStampedConstPtr& msg) {

}

void ParticleFilter::imuCallback(sensor_msgs::ImuConstPtr& msg) {
  // if it is the first message use the initial time
  if(last_imu_time_.toSec() == 0) {
    last_imu_time_ = msg->header.stamp;
    return;
  }

  double dt = msg->header.stamp.toSec() - last_imu_time_.toSec();
  last_imu_time_ = msg->header.stamp;



}

void ParticleFilter::imuMotionUpdate(const geometry_msgs::Vector3 linear_acc,
                                     const geometry_msgs::Vector3& angular_vel, const double dt) {
  for(int i = 0; i < num_particles_; i++) {
    propagateParticle(particles_[i], linear_acc, angular_vel, dt);
  }
}

void ParticleFilter::propagateParticle(Particle& p, const geometry_msgs::Vector3& linear_acc,
                                       const geometry_msgs::Vector3& angular_vel, const double dt) {
  p.x = p.x + p.vx*dt + sigma_x_*distribution_(generator_)*sqrt(dt);
  p.y = p.y + p.vy*dt + sigma_y_*distribution_(generator_)*sqrt(dt);

  p.vx = p.vx + cos(p.yaw)*linear_acc.x*dt + sigma_vx_*distribution_(generator_)*sqrt(dt);
  p.vy = p.vy + sin(p.yaw)*linear_acc.y*dt + sigma_vy_*distribution_(generator_)*sqrt(dt);

  p.yaw = p.yaw + angular_vel.z*dt + sigma_yaw_*distribution_(generator_)*sqrt(dt);
}
