//
// Created by jason on 7/5/21.
//

#ifndef SRC_MOTION_MODEL_H
#define SRC_MOTION_MODEL_H

#include "particle.h"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "random"

namespace localization {

class IMUMotionModel {
public:
  IMUMotionModel(std::shared_ptr<ParticleNoise> noise, rclcpp::Node* node);

  void updateParticle(Particle & particle, double dt, geometry_msgs::msg::Twist::SharedPtr cmd_msg);
  void updateParticles(std::vector<Particle> & particles,
                       geometry_msgs::msg::Twist::SharedPtr cmd_msg,
                       rclcpp::Time current_time);
  void ImuCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
private:
  Particle sigmas_;
  rclcpp::Time last_message_time_;

  std::shared_ptr<ParticleNoise> noise_;
};

}


#endif //SRC_MOTION_MODEL_H
