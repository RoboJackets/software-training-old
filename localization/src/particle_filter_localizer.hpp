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

#ifndef PARTICLE_FILTER_LOCALIZER_HPP_
#define PARTICLE_FILTER_LOCALIZER_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <random>
#include <vector>

#include "particle.hpp"
#include "sensor_model.hpp"
#include "motion_model.hpp"
#include "aruco_sensor_model.hpp"
#include "odometry_sensor_model.hpp"

namespace localization
{

class ParticleFilterLocalizer : public rclcpp::Node
{
public:
  explicit ParticleFilterLocalizer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr resample_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::vector<localization::Particle> particles_;
  std::vector<double> search_weights_;

  std::shared_ptr<ParticleNoise> noise_;
  std::vector<std::unique_ptr<SensorModel>> sensor_models_;
  IMUMotionModel motion_model_;

  int num_particles_ = 100;
  double max_weight_ = 0;
  double min_weight_ = 0;

  Particle min_;
  Particle max_;

  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  Particle InitializeParticle();
  void NormalizeWeights();
  void ResampleParticles();
  void CalculateStateAndPublish();
  void ComputeLogProbs();

  void PublishParticleVisualization();
};

}  // namespace localization


#endif  // PARTICLE_FILTER_LOCALIZER_HPP_
