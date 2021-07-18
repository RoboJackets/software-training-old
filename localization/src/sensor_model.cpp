#include "sensor_model.h"

namespace localization {

  void SensorModel::ComputeLogProbs(std::vector<Particle> & particles, rclcpp::Time cur_time,
                               SensorModel & aruco_model, SensorModel & odom_model)
  {
    for(Particle & particle : particles)
    {
      double log_prob = 0;
      if (aruco_model.IsMeasUpdateValid(cur_time))
      {
        log_prob += -0.5*aruco_model.ComputeLogProb(particle);
        log_prob -= aruco_model.ComputeLogNormalizer();
      }

      if(odom_model.IsMeasUpdateValid(cur_time))
      {
        log_prob += -0.5*odom_model.ComputeLogProb(particle);
        log_prob -= odom_model.ComputeLogNormalizer();
      }

      particle.weight = exp(log_prob);
      //std::cout << "particle weight: " << particle.weight << std::endl;
    }
  }

  bool SensorModel::IsMeasUpdateValid(rclcpp::Time cur_time)
  {
    return true;
  }
}