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

#include <angles/angles.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <nav2_core/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "controller_helpers.h"

namespace controllers
{

class PIDController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = node;

    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    traj_viz_pub_ = node_shared->create_publisher<nav_msgs::msg::Path>(
      "~/tracking_traj",
      rclcpp::SystemDefaultsQoS());

    // BEGIN STUDENT CODE
    time_between_states_ =
      node_shared->declare_parameter<double>(name + ".time_between_states", 3.0);

    bx_P_ = node_shared->declare_parameter<double>(name + ".bx.P", 1.0);
    bx_I_ = node_shared->declare_parameter<double>(name + ".bx.I", 0.0);
    bx_D_ = node_shared->declare_parameter<double>(name + ".bx.D", 0.0);

    by_P_ = node_shared->declare_parameter<double>(name + ".by.P", 1.0);
    by_I_ = node_shared->declare_parameter<double>(name + ".by.I", 0.0);
    by_D_ = node_shared->declare_parameter<double>(name + ".by.D", 0.0);

    yaw_P_ = node_shared->declare_parameter<double>(name + ".yaw.P", 1.0);
    yaw_I_ = node_shared->declare_parameter<double>(name + ".yaw.I", 0.0);
    yaw_D_ = node_shared->declare_parameter<double>(name + ".yaw.D", 0.0);

    prev_error_ = Eigen::Vector3d::Zero();
    integral_error_ = Eigen::Vector3d::Zero();
    integral_max_ = node_shared->declare_parameter<std::vector<double>>(
      name + ".integral_max", {1.0, 1.0, 1.0});
    if (integral_max_.size() != 3) {
      RCLCPP_ERROR(node_shared->get_logger(), "incorrect size integral_max, must be 3 values");
      exit(0);
    }

    // END STUDENT CODE
  }

  Eigen::Vector3d interpolateState(const rclcpp::Time time)
  {
    if (time.seconds() > path_start_time_.seconds() + (time_between_states_ * trajectory_.size())) {
      return trajectory_.back();
    }
    if (time < path_start_time_) {
      return trajectory_.front();
    }

    double rel_time = (time - path_start_time_).seconds();
    int lower_idx = std::floor(rel_time / time_between_states_);
    int upper_idx = lower_idx + 1;
    double alpha = (rel_time - lower_idx * time_between_states_) / time_between_states_;

    Eigen::Vector3d interpolated = (1 - alpha) * trajectory_[lower_idx] + alpha *
      trajectory_[upper_idx];

    // correct the angle average
    if ((trajectory_[lower_idx](2) > 0 && trajectory_[upper_idx](2) < 0 ||
      trajectory_[lower_idx](2) < 0 && trajectory_[upper_idx](2) > 0) &&
      (std::abs(trajectory_[lower_idx](2)) > M_PI_2 && std::abs(
        trajectory_[upper_idx](2)) > M_PI_2))
    {
      double angle_diff = angles::shortest_angular_distance(
        trajectory_[lower_idx](2), trajectory_[upper_idx](2));
      interpolated(2) = trajectory_[lower_idx](2) + alpha * angle_diff;
      interpolated(2) = angles::normalize_angle(interpolated(2));
    }

    return interpolated;
  }

  void activate() override
  {
    traj_viz_pub_->on_activate();
  }

  void deactivate() override {}

  void cleanup() override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    trajectory_.clear();
    std::transform(
      path.poses.begin(), path.poses.end(), std::back_inserter(
        trajectory_), StateFromMsg);
    path_start_time_ = node_shared->now();

    prev_error_ = Eigen::Vector3d::Zero();
    integral_error_ = Eigen::Vector3d::Zero();
    prev_time_ = 0;
  }

  void resetStates(Eigen::Vector3d init_state)
  {
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }
  }

  void computeError(
    const Eigen::Vector3d & state, const Eigen::Vector3d target_state, double dt,
    Eigen::Vector3d & error_delta, Eigen::Vector3d & error)
  {
    Eigen::Matrix3d R;
    R << cos(state(2)), -sin(state(2)), 0, sin(state(2)), cos(state(2)), 0, 0, 0, 1;

    // BEGIN STUDENT CODE
    error = target_state - state;
    error(2) = angles::shortest_angular_distance(state(2), target_state(2));

    error = R.transpose() * error;

    error_delta = (error - prev_error_) / dt;
    integral_error_ += error_delta * dt;
    // END STUDENT CODE
  }

  double computePID(
    double error, double error_delta, double integral_error,
    double P, double D, double I)
  {
    // BEGIN STUDENT CODE
    return error * P + error_delta * D + integral_error * I;
    // END STUDENT CODE
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // TODO(barulicm) goal_checker added in humble
    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    if (prev_time_ == 0) {
      // on the first call return a zero control to get a dt estimate
      prev_time_ = node_shared->now().seconds();
      geometry_msgs::msg::TwistStamped cmd_vel_msg;
      cmd_vel_msg.twist.linear.x = 0;
      cmd_vel_msg.twist.angular.z = 0;
      cmd_vel_msg.header.frame_id = "base_link";
      cmd_vel_msg.header.stamp = node_shared->now();
      return cmd_vel_msg;
    }

    Eigen::Vector3d state = StateFromMsg(pose);
    Eigen::Vector3d target_state = interpolateState(pose.header.stamp);
    double dt = node_shared->now().seconds() - prev_time_;

    Eigen::Vector3d error, error_delta;
    computeError(state, target_state, dt, error_delta, error);

    // clamps the maximum value of the integral
    for (int i = 0; i < 3; i++) {
      integral_error_(i) = std::clamp(integral_error_(i), -integral_max_[i], integral_max_[i]);
    }

    double bx_pid = computePID(error(0), error_delta(0), integral_error_(0), bx_P_, bx_D_, bx_I_);
    double by_pid = computePID(error(1), error_delta(1), integral_error_(1), by_P_, by_D_, by_I_);
    double yaw_pid =
      computePID(error(2), error_delta(2), integral_error_(2), yaw_P_, yaw_D_, yaw_I_);

    // setup variables for the next iteration
    error = prev_error_;
    prev_time_ = node_shared->now().seconds();

    // convert the pid components into the controls
    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.linear.x = std::clamp(bx_pid, -2.0, 2.0);
    cmd_vel_msg.twist.angular.z = std::clamp(yaw_pid + by_pid, -2.0, 2.0);
    cmd_vel_msg.header.frame_id = "base_link";
    cmd_vel_msg.header.stamp = node_shared->now();
    return cmd_vel_msg;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    // TODO(barulicm) implement this
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  // dynamics

  double bx_P_;
  double bx_D_;
  double bx_I_;

  double by_P_;
  double by_D_;
  double by_I_;

  double yaw_P_;
  double yaw_D_;
  double yaw_I_;

  Eigen::Vector3d prev_error_;
  Eigen::Vector3d integral_error_;
  std::vector<double> integral_max_;

  double prev_time_;

  // trajectory to track
  std::vector<Eigen::Vector3d> trajectory_;
  double time_between_states_;
  rclcpp::Time path_start_time_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_viz_pub_;
};

}  // namespace controllers

PLUGINLIB_EXPORT_CLASS(controllers::PIDController, nav2_core::Controller)
