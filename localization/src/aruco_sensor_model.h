//
// Created by jason on 7/9/21.
//

#ifndef SRC_ARUCO_SENSOR_MODEL_H
#define SRC_ARUCO_SENSOR_MODEL_H

#include "sensor_model.h"
#include <stsl_interfaces/msg/tag_array.hpp>

namespace localization {

struct TagLocation {
  double x = 0;
  double y = 0;
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
