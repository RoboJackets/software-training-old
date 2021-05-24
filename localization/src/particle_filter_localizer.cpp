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
#include <stsl_interfaces/msg/tag_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace localization
{
class ParticleFilterLocalizer : public rclcpp::Node
{
public:
  explicit ParticleFilterLocalizer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("particle_filter_localizer", options),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(*this)
  {
    tag_subscription_ = create_subscription<stsl_interfaces::msg::TagArray>(
      "/tags", 1, std::bind(&ParticleFilterLocalizer::TagCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ParticleFilterLocalizer::OdometryCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void TagCallback(const stsl_interfaces::msg::TagArray::SharedPtr msg)
  {
  }

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
  }
};

}  // namespace localization
