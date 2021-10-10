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
