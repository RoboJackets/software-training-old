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
    Eigen::Vector3d state = StateFromMsg(pose);

    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;


    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.frame_id = "base_link";
    cmd_vel_msg.header.stamp = node_->now();
    return cmd_vel_msg;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::vector<Eigen::Vector3d> trajectory_;

};

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LqrController, nav2_core::Controller)
