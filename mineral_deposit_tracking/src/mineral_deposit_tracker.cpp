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

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/mineral_deposit_array.hpp>
#include <stsl_interfaces/srv/reset_mineral_deposit_tracking.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "kalman_filter.hpp"

namespace mineral_deposit_tracking
{

class MineralDepositTracker : public rclcpp::Node
{
public:
  explicit MineralDepositTracker(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mineral_deposit_tracker", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    filter_(Eigen::Matrix2d::Identity(),
      Eigen::Matrix2d::Identity() * 1e-4, Eigen::Matrix2d::Identity())
  {
    tracked_deposit_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/tracked_deposit", rclcpp::SystemDefaultsQoS());
    deposit_subscription_ = create_subscription<stsl_interfaces::msg::MineralDepositArray>(
      "~/sensed_deposits", rclcpp::SystemDefaultsQoS(),
      std::bind(&MineralDepositTracker::DepositMeasurementCallback, this, std::placeholders::_1));
    reset_service_ = create_service<stsl_interfaces::srv::ResetMineralDepositTracking>(
      "~/reset", std::bind(
        &MineralDepositTracker::ResetCallback, this, std::placeholders::_1,
        std::
        placeholders::_2));
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_deposit_publisher_;
  rclcpp::Subscription<stsl_interfaces::msg::MineralDepositArray>::SharedPtr deposit_subscription_;
  rclcpp::Service<stsl_interfaces::srv::ResetMineralDepositTracking>::SharedPtr reset_service_;
  int deposit_id_;
  KalmanFilter filter_;

  void DepositMeasurementCallback(const stsl_interfaces::msg::MineralDepositArray::SharedPtr msg)
  {
    try {
      filter_.TimeUpdate();
      const auto found_deposit = std::find_if(
        msg->deposits.begin(), msg->deposits.end(), [this](const auto & deposit) {
          return deposit.id == deposit_id_;
        });
      if (found_deposit == msg->deposits.end()) {
        return;
      }
      const auto & detection = *found_deposit;

      const Eigen::Vector3d sensor_frame_position = detection.range * Eigen::Vector3d{std::cos(
          detection.heading), std::sin(detection.heading), 0.0};

      const auto transform =
        tf_buffer_.lookupTransform("map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

      Eigen::Vector3d map_frame_position;

      tf2::doTransform(sensor_frame_position, map_frame_position, transform);

      const Eigen::Vector2d measurement{map_frame_position.x(), map_frame_position.y()};

      const Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity() * 0.01;

      filter_.MeasurementUpdate(measurement, covariance);

      const auto estimated_position = filter_.GetEstimate();

      geometry_msgs::msg::PoseStamped output_msg;
      output_msg.header.stamp = now();
      output_msg.header.frame_id = "map";
      output_msg.pose.position.x = estimated_position.x();
      output_msg.pose.position.y = estimated_position.y();
      tracked_deposit_publisher_->publish(output_msg);
    } catch (const tf2::TransformException& e) {
      RCLCPP_ERROR(get_logger(), "TF Error: %s", e.what());
    }
  }

  void ResetCallback(
    const stsl_interfaces::srv::ResetMineralDepositTracking::Request::SharedPtr request,
    stsl_interfaces::srv::ResetMineralDepositTracking::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Received reset request!");
    deposit_id_ = request->id;
    const Eigen::Vector2d position{request->pose.pose.position.x, request->pose.pose.position.y};
    Eigen::Matrix2d covariance;
    covariance << request->pose.covariance[0], request->pose.covariance[1],
      request->pose.covariance[3], request->pose.covariance[4];
    filter_.Reset(position, covariance);
  }
};

}  // namespace mineral_deposit_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(mineral_deposit_tracking::MineralDepositTracker)
