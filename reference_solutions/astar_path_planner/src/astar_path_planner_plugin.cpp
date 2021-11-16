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

#include "astar_path_planner.hpp"
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <memory>
#include <string>
#include <vector>

namespace astar_path_planner
{

class AStarPathPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = node;
    global_frame_ = costmap_ros->getGlobalFrameID();
    costmap_ros_ = costmap_ros;
    AStarPathPlanner::DeclareParameters(node);
    expanded_viz_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
      "~/expanded_viz",
      rclcpp::SystemDefaultsQoS());
  }

  void cleanup() override {}

  void activate() override
  {
    expanded_viz_pub_->on_activate();
  }

  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
      return path;
    }

    if (goal.header.frame_id != global_frame_) {
      RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
      return path;
    }

    Point start_point{start.pose.position.x, start.pose.position.y};
    Point goal_point{goal.pose.position.x, goal.pose.position.y};

    RCLCPP_INFO(
      node_->get_logger(), "Calculating path from (%f, %f) to (%f, %f)",
      start_point.x(), start_point.y(), goal_point.x(), goal_point.y());

    AStarPathPlanner planner(node_, costmap_ros_);

    const auto point_path = planner.Plan(start_point, goal_point);

    PublishExpandedViz(planner.GetExpandedSet());

    if (point_path.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Could not find path!");
      return path;
    }

    RCLCPP_INFO(node_->get_logger(), "Calculated path with %d points.", point_path.size());

    auto point_to_message = [prev = start_point](const auto & point) mutable {
        const Eigen::Vector2d heading_vector = point - prev;
        const auto heading = std::atan2(heading_vector.y(), heading_vector.x());
        const Eigen::Quaterniond orientation{Eigen::AngleAxisd{heading, Eigen::Vector3d::UnitZ()}};
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.orientation = tf2::toMsg(orientation);
        prev = point;
        return pose;
      };

    std::transform(
      point_path.begin(), point_path.end(),
      std::back_inserter(path.poses), point_to_message);

    path.poses.front().pose.orientation = start.pose.orientation;

    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string global_frame_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr expanded_viz_pub_;


  void PublishExpandedViz(const AStarPathPlanner::ExpandedSet & expanded)
  {
    visualization_msgs::msg::Marker msg;

    msg.header.frame_id = "map";
    msg.header.stamp = node_->now();
    msg.ns = "/planner/expanded";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.action = visualization_msgs::msg::Marker::ADD;

    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;

    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 0.33;

    std::transform(
      expanded.begin(), expanded.end(), std::back_inserter(msg.points), [](const auto & p) {
        geometry_msgs::msg::Point out;
        out.x = p.x();
        out.y = p.y();
        return out;
      });

    expanded_viz_pub_->publish(msg);
  }
};

}  // namespace astar_path_planner

PLUGINLIB_EXPORT_CLASS(astar_path_planner::AStarPathPlannerPlugin, nav2_core::GlobalPlanner)
