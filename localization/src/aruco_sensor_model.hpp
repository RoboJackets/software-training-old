// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef ARUCO_SENSOR_MODEL_HPP_
#define ARUCO_SENSOR_MODEL_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <map>
#include "sensor_model.hpp"

namespace localization
{

struct TagLocation
{
  double x = 0;
  double y = 0;
  double yaw = 0;
};

class ArucoSensorModel : public SensorModel
{
public:
  explicit ArucoSensorModel(rclcpp::Node & node);
  void UpdateMeasurement(const stsl_interfaces::msg::TagArray::SharedPtr msg);
  double ComputeLogProb(const Particle & particle) override;
  double ComputeLogNormalizer() override;
  bool IsMeasurementAvailable(const rclcpp::Time & cur_time) override;

private:
  stsl_interfaces::msg::TagArray last_msg_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  std::map<int, TagLocation> tags_;
};
}  // namespace localization


#endif  // ARUCO_SENSOR_MODEL_HPP_
