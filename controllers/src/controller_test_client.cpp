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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include "test_path_generator.hpp"

namespace controllers
{

class ControllerTestClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit ControllerTestClient(const rclcpp::NodeOptions & options)
  : rclcpp::Node("controller_test_client", options)
  {
    client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    rclcpp::QoS qos_profile{1};
    qos_profile.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", qos_profile);

    std::thread(&ControllerTestClient::SendGoal, this).detach();
  }

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void SendGoal()
  {
    RCLCPP_INFO(get_logger(), "Waiting for /follow_path action server.");
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not ready. Waiting timed out.");
      rclcpp::shutdown();
    }

    FollowPath::Goal goal;
    goal.path = TestPathGenerator(20).BuildPath();
    goal.path.header.frame_id = "map";
    goal.path.header.stamp = now();

    path_pub_->publish(goal.path);

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
      &ControllerTestClient::GoalResponseCallback, this,
      std::placeholders::_1);
    options.feedback_callback = std::bind(
      &ControllerTestClient::FeedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(
      &ControllerTestClient::ResultCallback, this,
      std::placeholders::_1);
    client_->async_send_goal(goal, options);
  }

  void GoalResponseCallback(GoalHandle::SharedPtr goal_handle)
  {
    if (goal_handle) {
      RCLCPP_INFO(get_logger(), "Server accepted goal!");
    } else {
      RCLCPP_ERROR(get_logger(), "Server rejected goal!");
    }
  }

  void FeedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance to goal: %f", feedback->distance_to_goal);
  }

  void ResultCallback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Path following completed!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Path following aborted!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Path following canceled!");
        break;
      case rclcpp_action::ResultCode::UNKNOWN:
        RCLCPP_ERROR(get_logger(), "Unkown action result code!");
        break;
    }
    rclcpp::shutdown();
  }
};

}  // namespace controllers

RCLCPP_COMPONENTS_REGISTER_NODE(controllers::ControllerTestClient)
