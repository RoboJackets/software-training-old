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

#include "navigator.hpp"

namespace peak_finder
{
Navigator::Navigator(rclcpp::Node & node)
: node_(node)
{
  navigation_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(&node_, "/navigate_to_pose");
}

bool Navigator::server_available()
{
  return navigation_client_->action_server_is_ready();
}

bool Navigator::go_to_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (goal_handle_ && (goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
    goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING))
  {
    RCLCPP_ERROR(
      node_.get_logger(),
      "Navigator already working towards a goal. You need to cancel or wait for the "
      "previous goal.");
    return false;
  }
  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  navigation_goal.pose = pose;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_options;
  send_options.feedback_callback = std::bind(
    &Navigator::feedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  auto goal_handle_future = navigation_client_->async_send_goal(navigation_goal, send_options);
  goal_handle_future.wait();
  goal_handle_ = goal_handle_future.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_.get_logger(), "Navigation server rejected request.");
    return false;
  }
  result_future_ = navigation_client_->async_get_result(goal_handle_);
  return true;
}

bool Navigator::wait_for_completion(const std::chrono::nanoseconds & timeout)
{
  if (timeout.count() >= 0) {
    const auto status = result_future_.wait_for(timeout);
    return status == std::future_status::ready;
  } else {
    result_future_.wait();
    return true;
  }
}

bool Navigator::succeeded()
{
  if (!result_future_.valid()) {
    RCLCPP_WARN(node_.get_logger(), "Navigator::succeeded() called before navigation completed.");
    return false;
  }
  const auto wrapped_result = result_future_.get();
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

void Navigator::cancel()
{
  if (!goal_handle_ || (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING &&
    goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED))
  {
    RCLCPP_WARN(node_.get_logger(), "Navigator::cancel() called with no active goal.");
    return;
  }
  const auto status =
    navigation_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_.get_logger(), "Timed out waiting for navigation goal to cancel.");
  }
}

void Navigator::feedbackCallback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
  const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback)
{
  // RCLCPP_INFO(node_.get_logger(), "Feedback: %f", feedback->distance_remaining);
}

}  // namespace peak_finder
