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
#include <stsl_interfaces/action/park_at_peak.hpp>
#include <stsl_interfaces/srv/sample_elevation.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace peak_finder
{
class PeakFinderComponent : public rclcpp::Node
{
public:
  using ParkAtPeak = stsl_interfaces::action::ParkAtPeak;
  using ParkAtPeakGoalHandle = rclcpp_action::ServerGoalHandle<ParkAtPeak>;

  explicit PeakFinderComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("peak_finder", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    action_server_ = rclcpp_action::create_server<ParkAtPeak>(
        this, "park_at_peak",
        std::bind(&PeakFinderComponent::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&PeakFinderComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&PeakFinderComponent::handle_accepted, this, std::placeholders::_1));

    elevation_client_ = create_client<stsl_interfaces::srv::SampleElevation>("/sample_elevation");

    navigation_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<ParkAtPeak>::SharedPtr action_server_;
  rclcpp::Client<stsl_interfaces::srv::SampleElevation>::SharedPtr elevation_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                          std::shared_ptr<const ParkAtPeak::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    std::thread{ std::bind(&PeakFinderComponent::execute, this, std::placeholders::_1),
                 goal_handle }
        .detach();
  }

  void execute(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle)
  {
    if (!elevation_client_->service_is_ready())
    {
      RCLCPP_ERROR(get_logger(), "%s service must be available to run peak_finder action!",
                   elevation_client_->get_service_name());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    std::string tf_error_msg;
    if (!tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &tf_error_msg))
    {
      RCLCPP_ERROR(get_logger(), "Robot position could not be looked up via TF. Error: %s",
                   tf_error_msg.c_str());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    if (!navigation_client_->action_server_is_ready())
    {
      RCLCPP_ERROR(get_logger(),
                   "/navigate_to_point action must be available to run peak_finder action!");
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return;
    }

    try
    {
      while (rclcpp::ok() && !goal_handle->is_canceling())
      {
        RCLCPP_INFO(get_logger(), "Getting current position.");
        const auto robot_position = GetRobotPosition();

        double current_elevation;

        RCLCPP_INFO(get_logger(), "Sampling current elevations.");
        if(!SampleElevation(goal_handle, robot_position, current_elevation))
        {
          return;
        }

        RCLCPP_INFO(get_logger(), "Current elevation: %f", current_elevation);

        auto generate_next_pose = [&robot_position, angle = 0.0]() mutable {
          const auto look_distance = 0.2;
          Eigen::Vector2d pose =
              (look_distance * Eigen::Vector2d(std::cos(angle), std::sin(angle))) + robot_position;
          angle += M_PI_2;
          return pose;
        };

        std::vector<Eigen::Vector2d> sample_positions;

        std::generate_n(std::back_inserter(sample_positions), 4, generate_next_pose);

        std::vector<double> elevations;

        RCLCPP_INFO(get_logger(), "Sampling nearby elevations.");
        for(const auto& position : sample_positions)
        {
          double elevation;
          if(!SampleElevation(goal_handle, position, elevation)) {
            return;
          }
          elevations.push_back(elevation);
        }

        {
          std::stringstream ss;
          for(const auto& elevation : elevations) {
            ss << elevation << " ";
          }
          RCLCPP_INFO(get_logger(), "Elevations: %s", ss.str().c_str());
        }

        const auto max_elevation_iter = std::max_element(elevations.begin(), elevations.end());

        RCLCPP_INFO(get_logger(), "Max elevation: %f", *max_elevation_iter);

        if (*max_elevation_iter <= current_elevation)
        {
          RCLCPP_INFO(get_logger(), "At peak!");
          goal_handle->succeed(std::make_shared<ParkAtPeak::Result>());
          return;
        }

        const auto goal_position =
            sample_positions[std::distance(elevations.begin(), max_elevation_iter)];

        RCLCPP_INFO(get_logger(), "Moving to new position.");
        if (!MoveToPosition(goal_handle, goal_position))
        {
          return;
        }
      }
      goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
    }
  }

  Eigen::Vector2d GetRobotPosition()
  {
    const auto robot_pose_transform =
        tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    Eigen::Isometry3d robot_pose_3d = tf2::transformToEigen(robot_pose_transform);
    return robot_pose_3d.translation().head<2>();
  }

  bool SampleElevation(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle, const Eigen::Vector2d& position, double & elevation)
  {
    RCLCPP_INFO(get_logger(), "Sending sample request at <%f, %f>", position.x(), position.y());
    auto sample_request = std::make_shared<stsl_interfaces::srv::SampleElevation::Request>();
    sample_request->x = position.x();
    sample_request->y = position.y();
    const auto result_future = elevation_client_->async_send_request(sample_request);

    stsl_interfaces::srv::SampleElevation::Response::SharedPtr response;

    RCLCPP_INFO(get_logger(), "Waiting for response.");
    while(true)
    {
      if(!rclcpp::ok() || goal_handle->is_canceling())
      {
        goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
        return false;
      }
      const auto status = result_future.wait_for(std::chrono::milliseconds(100));
      if(status == std::future_status::ready)
      {
        response = result_future.get();
        break;
      }
    }
    RCLCPP_INFO(get_logger(), "Elevation response received.");

    if(!response->success)
    {
      RCLCPP_ERROR(get_logger(), "Elevation server reported failure.");
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return false;
    }

    elevation = response->elevation;
    return true;
  }

  bool MoveToPosition(const std::shared_ptr<ParkAtPeakGoalHandle> goal_handle,
                      const Eigen::Vector2d& position)
  {
    RCLCPP_INFO(get_logger(), "Sending navigation goal.");
    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    navigation_goal.pose.header.frame_id = "map";
    navigation_goal.pose.header.stamp = now();
    navigation_goal.pose.pose.position.x = position.x();
    navigation_goal.pose.pose.position.y = position.y();
    navigation_goal.pose.pose.position.z = 0.0;
    navigation_goal.pose.pose.orientation.x = 0.0;
    navigation_goal.pose.pose.orientation.y = 0.0;
    navigation_goal.pose.pose.orientation.z = 0.0;
    navigation_goal.pose.pose.orientation.w = 0.0;
    const auto nav_goal_handle_future = navigation_client_->async_send_goal(navigation_goal);

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle;

    RCLCPP_INFO(get_logger(), "Waiting for acceptance.");
    while (true)
    {
      if (!rclcpp::ok() || goal_handle->is_canceling())
      {
        goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
        return false;
      }
      const auto status = nav_goal_handle_future.wait_for(std::chrono::milliseconds(100));
      if (status == std::future_status::ready)
      {
        nav_goal_handle = nav_goal_handle_future.get();
        break;
      }
    }

    if (!nav_goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Navigation stack reject move request.");
      goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Waiting for navigation result.");
    rclcpp::Rate rate{ 10 /*Hz*/ };
    while (true)
    {
      if (!rclcpp::ok() || goal_handle->is_canceling())
      {
        goal_handle->canceled(std::make_shared<ParkAtPeak::Result>());
        return false;
      }
      const auto status = nav_goal_handle->get_status();
      switch (status)
      {
        case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
        case rclcpp_action::GoalStatus::STATUS_EXECUTING:
          continue;
        case rclcpp_action::GoalStatus::STATUS_ABORTED:
        case rclcpp_action::GoalStatus::STATUS_CANCELED:
        case rclcpp_action::GoalStatus::STATUS_CANCELING:
        case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
          RCLCPP_ERROR(get_logger(), "Navigation failed.");
          goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
          return false;
        case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Navigation succeeded.");
          return true;
      }
      rate.sleep();
    }
  }
};

}  // namespace peak_finder

RCLCPP_COMPONENTS_REGISTER_NODE(peak_finder::PeakFinderComponent)
