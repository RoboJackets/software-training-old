#ifndef SRC_PARTICLE_FILTER_LOCALIZER_H
#define SRC_PARTICLE_FILTER_LOCALIZER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "particle.h"
#include "sensor_model.h"
#include "motion_model.h"
#include "aruco_sensor_model.h"
#include "odometry_sensor_model.h"
#include "random"

namespace localization
{

class ParticleFilterLocalizer : public rclcpp::Node
{
public:
  explicit ParticleFilterLocalizer(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
  ArucoSensorModel aruco_model_;
  OdometrySensorModel odom_model_;
  IMUMotionModel motion_model_;

  int num_particles_ = 100;
  double max_weight_ = 0;
  double min_weight_ = 0;

  Particle min_;
  Particle max_;

  void TagCallback(const stsl_interfaces::msg::TagArray::SharedPtr msg);
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  Particle InitializeParticle();
  void NormalizeWeights();
  void ResampleParticles();
  void CalculateStateAndPublish();

  void PublishParticleVisualization();
};

} // namespace localization


#endif //SRC_PARTICLE_FILTER_LOCALIZER_H
