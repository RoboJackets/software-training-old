//
// Created by jason on 7/9/21.
//

#ifndef SRC_ODOMETRY_SENSOR_MODEL_H
#define SRC_ODOMETRY_SENSOR_MODEL_H

#include "sensor_model.h"
#include <nav_msgs/msg/odometry.hpp>

namespace localization {
class OdometrySensorModel : public SensorModel {
public:
  OdometrySensorModel(rclcpp::Node* node);

  void UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr & odom);
  double ComputeLogProb(Particle & particle) override;
  double ComputeLogNormalizer() override;
  bool IsMeasUpdateValid(rclcpp::Time cur_time) override;
private:
  nav_msgs::msg::Odometry::SharedPtr last_msg_;

};
}


#endif //SRC_ODOMETRY_SENSOR_MODEL_H
