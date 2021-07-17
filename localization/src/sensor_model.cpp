#include "sensor_model.h"

namespace localization {

  static void ComputeLogProbs(std::vector<Particle> & particles, rclcpp::Time cur_time,
                               SensorModel & aruco_model, SensorModel & odom_model)
  {
    for(Particle & particle : particles)
    {
      double log_prob = 0;
      log_prob += aruco_model.ComputeLogProb(particle);
      log_prob *= -0.5;
      log_prob -= aruco_model.ComputeLogNormalizer();

      log_prob += odom_model.ComputeLogProb(particle);
      log_prob *= -0.5;
      log_prob -= odom_model.ComputeLogNormalizer();

      particle.weight = exp(log_prob);
    }
  }

  bool SensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
  {
    return true;
  }
}