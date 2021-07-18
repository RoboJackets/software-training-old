//
// Created by jason on 7/9/21.
//

#ifndef SRC_ARUCO_SENSOR_MODEL_H
#define SRC_ARUCO_SENSOR_MODEL_H

#include "sensor_model.h"
#include <stsl_interfaces/msg/tag_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace localization {

struct TagLocation {
  TagLocation() {}
  TagLocation(double x, double y, double yaw) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
  }
  double x = 0;
  double y = 0;
  double yaw = 0;
};

class ArucoSensorModel : public SensorModel {
public:
  ArucoSensorModel(rclcpp::Node* node);
  void UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr & tags);
  double ComputeLogProb(Particle & particle) override;
  double ComputeLogNormalizer() override;
  bool IsMeasUpdateValid(rclcpp::Time cur_time) override;
private:
  stsl_interfaces::msg::TagArray::SharedPtr last_msg_;
  std::map<int, TagLocation> tags_;
};
}


#endif //SRC_ARUCO_SENSOR_MODEL_H
