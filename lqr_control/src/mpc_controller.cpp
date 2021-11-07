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

#include <nav2_core/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

namespace mpc_controller
{

Eigen::Vector3d StateFromMsg(const geometry_msgs::msg::PoseStamped& pose) {
  Eigen::Quaterniond orientation;
  tf2::fromMsg(pose.pose.orientation, orientation);
  Eigen::Vector3d state;
  state << pose.pose.position.x, pose.pose.position.y, orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];
  return state;
}

class MpcController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap) override {
    node_ = node;

    T_ = node->declare_parameter<double>("T", 1.0);
    dt_ = node->declare_parameter<double>("dt", 0.1);
    samples_ = node->declare_parameter<double>("samples", 100);
    std::vector<double> Q_temp = node->declare_parameter<std::vector<double>>("Q", {1.0, 1.0, 1.0});
    std::vector<double> Qf_temp = node->declare_parameter<std::vector<double>>("Qf", {1.0, 1.0, 1.0});
    std::vector<double> R_temp = node->declare_parameter<std::vector<double>>("R", {1.0, 1.0});
    if(Q_temp.size() != 3) {
      RCLCPP_ERROR(node_->get_logger(), "incorrect size Q, must be 3 values");
      exit(0);
    }
    if(Qf_temp.size() != 3) {
      RCLCPP_ERROR(node_->get_logger(), "incorrect size Qf, must be 3 values");
      exit(0);
    }
    if(R_temp.size() != 2) {
      RCLCPP_ERROR(node_->get_logger(), "incorrect size R, must be 2 values");
      exit(0);
    }

    // set the diagonal elements from the launch file
    Q_(0, 0) = Q_temp[0];
    Q_(1, 1) = Q_temp[1];
    Q_(2, 2) = Q_temp[2];

    Qf_(0, 0) = Qf_temp[0];
    Qf_(1, 1) = Qf_temp[1];
    Qf_(2, 2) = Qf_temp[2];

    R_(0, 0) = R_temp[0];
    R_(1, 1) = R_temp[1];

    prev_u_.resize(T_/dt_);
  }

  void activate() override {}

  void deactivate() override {}

  void cleanup() override {}

  void setPlan(const nav_msgs::msg::Path & path) override {
    trajectory_.clear();
    std::transform(path.poses.begin(), path.poses.end(), std::back_inserter(trajectory_), StateFromMsg);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity) override
  {
    RCLCPP_INFO(node_->get_logger(), "Got request for control");
    Eigen::Vector3d state = StateFromMsg(pose);

    Eigen::Vector2d u = sampleTrajectories(state);

    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.linear.x = u(0);
    cmd_vel_msg.twist.angular.z = u(1);
    cmd_vel_msg.header.frame_id = "base_link";
    cmd_vel_msg.header.stamp = node_->now();
    return cmd_vel_msg;
  }

  Eigen::Matrix3d computeAMatrix(const Eigen::Vector3d& x, const Eigen::Vector2d& u) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix<double, 3, 2> computeBMatrix(const Eigen::Vector3d& x) {
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = cos(x(2)) * dt_;
    B(1, 0) = sin(x(2)) * dt_;
    B(2, 1) = dt_;
    return B;
  }

  double computeRunningCost(const Eigen::Vector3d x, const Eigen::Vector2d u) {
    double state = x.transpose() * Q_ * x;
    double control = u.transpose() * R_ * u;
    return 0.5 * state + 0.5 * control;
  }

  double computeTerminalCost(const Eigen::Vector3d xf) {
    return 0.5 * xf.transpose() * Qf_ * xf;
  }

  Eigen::Vector3d computeNextState(const Eigen::Vector3d x, const Eigen::Vector2d u) {
    return computeAMatrix(x, u) * x + computeBMatrix(x) * u;
  }

  Eigen::Vector2d sampleTrajectories(const Eigen::Vector3d x_init) {
    double lowest_cost = 1e10;
    std::vector<Eigen::Vector2d> u_star;
    for(int sample = 0; sample < samples_; sample++) {
      double cost = 0;
      Eigen::Vector3d x = x_init;
      std::vector<Eigen::Vector2d> controls;

      Eigen::Vector2d u = prev_u_[0] + Eigen::Vector2d::Random().cwiseProduct(control_var_);
      controls.push_back(u);
      for(int t = 0; t < T_/dt_-1; t++) {
        cost += computeRunningCost(x - trajectory_[t], u);
        x = computeNextState(x - trajectory_[t], u);
        u = prev_u_[t+1] + Eigen::Vector2d::Random().cwiseProduct(control_var_);
        controls.push_back(u);
      }
      cost += computeTerminalCost(x);
      if(cost < lowest_cost) {
        lowest_cost = cost;
        u_star = controls;
      }
    }
    prev_u_ = u_star;
    return u_star[0];
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // dynamics
  double dt_ = 0.1;
  double T_ = 1;
  double samples_ = 100;

  // cost function
  Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Qf_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();

  // importance sampler
  std::vector<Eigen::Vector2d> prev_u_;
  Eigen::Vector2d control_var_;

  // trajectory to track
  std::vector<Eigen::Vector3d> trajectory_;

};

}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, nav2_core::Controller)
