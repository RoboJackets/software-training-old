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

namespace lqr_controller
{

Eigen::Vector3d StateFromMsg(const geometry_msgs::msg::PoseStamped& pose) {
  Eigen::Quaterniond orientation;
  tf2::fromMsg(pose.pose.orientation, orientation);
  Eigen::Vector3d state;
  state << pose.pose.position.x, pose.pose.position.y, orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];
  return state;
}

class LqrController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap) override {
    node_ = node;

    T_ = node->declare_parameter<double>("T", 1.0);
    dt_ = node->declare_parameter<double>("dt", 0.1);

    S_.resize(T_/dt_);
    prev_u_.resize(T_/dt_);
    prev_x_.resize(T_/dt_);
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

    computeRicattiEquation(state);

    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.linear.x = prev_u_[0](0);
    cmd_vel_msg.twist.angular.z = prev_u_[0](1);
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

  //double computeRunningCost(const Eigen::Vector3d x, const Eigen::Vector2d u) {
  //  double state = x.transpose() * Q_ * x;
  //  double control = u.transpose() * R_ * u;
  //  return 0.5 * state + 0.5 * control;
  //}

  //double computeTerminalCost(const Eigen::Vector3d xf) {
  //  return 0.5 * xf.transpose() * Qf_ * xf;
  //}

  Eigen::Vector3d computeNextState(const Eigen::Vector3d x, const Eigen::Vector2d u) {
    return computeAMatrix(x, u) * x + computeBMatrix(x) * u;
  }

  void computeRicattiEquation(const Eigen::Vector3d& init_x) {
    // does the backwards pass of the ricatti equation
    S_[S_.size()-1] = Qf_;
    for(int t = T_/dt_ - 1; t > 0; t--) {
      auto last_S = S_[t+1];
      Eigen::Matrix3d A = computeAMatrix(prev_x_[t], prev_u_[t]);
      Eigen::Matrix<double, 3, 2> B = computeBMatrix(prev_x_[t]);
      auto K = (R_ + B.transpose() * last_S * B).inverse() * B.transpose() * last_S * A;
      S_[t] = A.transpose() * last_S * A - (A.transpose() * last_S * B) * K + Q_;
    }

    // computes the forward pass to update the states and controls
    Eigen::Vector3d cur_x = init_x;
    for(int t = 0; t < T_/dt_; t++) {
      Eigen::Matrix3d A = computeAMatrix(cur_x, prev_u_[t]);
      Eigen::Matrix<double, 3, 2> B = computeBMatrix(prev_x_[t]);
      auto current_S = S_[t];
      auto K = (R_ + B.transpose() * current_S * B).inverse() * B.transpose() * current_S * A;
      Eigen::Vector2d u_star = -K * (cur_x - trajectory_[t]);

      prev_x_[t] = cur_x;
      prev_u_[t] = u_star;

      cur_x = computeNextState(cur_x, u_star);
    }
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // dynamics
  double dt_ = 0.1;
  double T_ = 1;

  // cost function
  Eigen::Matrix3d Q_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Qf_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity()*0.01;

  // Ricatti equation
  std::vector<Eigen::Matrix3d> S_;
  std::vector<Eigen::Vector3d> prev_x_;
  std::vector<Eigen::Vector2d> prev_u_;

  // trajectory to track
  std::vector<Eigen::Vector3d> trajectory_;

};

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LqrController, nav2_core::Controller)
