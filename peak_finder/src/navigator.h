#ifndef PEAK_FINDER_NAVIGATOR_H
#define PEAK_FINDER_NAVIGATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace peak_finder
{
class Navigator
{
public:
  Navigator(rclcpp::Node& node);

  bool server_available();

  bool go_to_pose(const geometry_msgs::msg::PoseStamped& pose);

  bool wait_for_completion(const std::chrono::nanoseconds& timeout = std::chrono::nanoseconds(-1));

  bool succeeded();

  void cancel();

private:
  rclcpp::Node& node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> result_future_;

  void feedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback);
};

}  // namespace peak_finder

#endif
