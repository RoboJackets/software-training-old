#ifndef SRC_SENSOR_MODEL_H
#define SRC_SENSOR_MODEL_H

#include "particle.h"
#include <stsl_interfaces/msg/tag_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "random"
#include <map>

namespace localization {


class SensorModel
{
public:
  virtual double ComputeLogProb(Particle & particle) {return 0.0;}
  virtual double ComputeLogNormalizer() {return 0.0;}
  virtual bool IsMeasUpdateValid(rclcpp::Time cur_time);
  ~SensorModel() = default;
protected:
  std::vector<double> meas_cov_;
  double time_delay_;
  SensorModel() = default;
private:
};
}


#endif //SRC_SENSOR_MODEL_H
