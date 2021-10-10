#include <nav2_core/controller.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace lqr_controller
{

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
    global_path_ = path;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity) override
  {
    
    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.frame_id = "base_link";
    cmd_vel_msg.header.stamp = node_->now();
    return cmd_vel_msg;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  nav_msgs::msg::Path global_path_;

};

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LqrController, nav2_core::Controller)
