//
// Created by jason on 10/17/20.
//
#pragma once

#include <ros/ros.h>
#include <random>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

struct Particle {
  double x = 0;
  double y = 0;
  double yaw = 0;
  double vx = 0;
  double vy = 0;
};

class ParticleFilter {
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher state_pub_;
  int num_particles_;
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  std::vector<Particle> particles_;

  double sigma_x_, sigma_y_, sigma_yaw_, sigma_vx_, sigma_vy_, sigma_yaw_rate_;

  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;

  ros::Time last_imu_time_ = ros::Time(0);


public:
  ParticleFilter() {
    // TODO grab params
    pNh = ros::NodeHandle("~");

    gps_sub_ = nh.subscribe("/oswin/fix", 1, )

    srand (100);

    initParticles();
  }

  void initParticles();

  void gpsCallback(geometry_msgs::PointStampedConstPtr& msg);
  void gpsSensorUpdate(geometry_msgs::PointStampedConstPtr& msg);

  void imuCallback(sensor_msgs::ImuConstPtr& msg);
  void imuMotionUpdate(const geometry_msgs::Vector3 linear_acc,
                       const geometry_msgs::Vector3& angular_vel, const double dt);
  void propagateParticle(Particle& p, const geometry_msgs::Vector3& linear_acc,
                         const geometry_msgs::Vector3& angular_vel, const double dt);

};

