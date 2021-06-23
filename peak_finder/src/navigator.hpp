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

#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <memory>

namespace peak_finder
{
class Navigator
{
public:
  explicit Navigator(rclcpp::Node & node);

  bool server_available();

  bool go_to_pose(const geometry_msgs::msg::PoseStamped & pose);

  bool wait_for_completion(const std::chrono::nanoseconds & timeout = std::chrono::nanoseconds(-1));

  bool succeeded();

  void cancel();

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  rclcpp::Node & node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  std::shared_future<GoalHandle::WrappedResult>
  result_future_;

  void feedbackCallback(
    std::shared_ptr<GoalHandle> goal_handle,
    const nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback);
};

}  // namespace peak_finder

#endif  // NAVIGATOR_HPP_
