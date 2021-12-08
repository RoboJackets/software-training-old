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
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <string>
// BEGIN STUDENT CODE
// END STUDENT CODE

using namespace std::chrono_literals;

namespace coordinate_transform
{
class CoordinateTransformComponent : public rclcpp::Node
{
public:
  explicit CoordinateTransformComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("coordinate_transformer", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    tag_sub_ = create_subscription<stsl_interfaces::msg::TagArray>(
      "~/tags",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CoordinateTransformComponent::DetectionCallback, this, std::placeholders::_1));
    tag_pub_ = create_publisher<stsl_interfaces::msg::TagArray>(
      "~/tags_transformed",
      rclcpp::SystemDefaultsQoS());
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  rclcpp::Publisher<stsl_interfaces::msg::TagArray>::SharedPtr tag_pub_;

  void DetectionCallback(const stsl_interfaces::msg::TagArray::SharedPtr tag_array_msg)
  {
    std::string tf_error_string;
    if (!tf_buffer_.canTransform(
        "base_footprint", tag_array_msg->header.frame_id,
        tag_array_msg->header.stamp, tf2::durationFromSec(0.1),
        &tf_error_string))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Could not lookup transform. %s",
        tf_error_string);
      return;
    }

    // find the transform from the camera to base footprint
    const auto tf_transform =
      tf_buffer_.lookupTransform("base_footprint", "camera_link", tag_array_msg->header.stamp);
    const Eigen::Matrix4d camera_to_base_transform = tf2::transformToEigen(tf_transform).matrix();

    // creates a matrix that goes from camera to standard ROS coordinates
    Eigen::Matrix4d camera_optical_to_conventional_transform =
      getTransformationMatrixForOpticalFrame();

    // BEGIN STUDENT CODE
    // END STUDENT CODE

    // create a new tag array message
    stsl_interfaces::msg::TagArray new_tag_array_msg;
    // copy the header information
    new_tag_array_msg.header.stamp = tag_array_msg->header.stamp;
    // change the frame_id to be the correct reference frame
    new_tag_array_msg.header.frame_id = "base_footprint";

    // BEGIN STUDENT CODE
    // set message tags to new_tags vector
    // END STUDENT CODE

    // publish new tag message
    tag_pub_->publish(new_tag_array_msg);
  }

  Eigen::Matrix4d getTransformationMatrixForOpticalFrame()
  {
    // BEGIN STUDENT CODE
    return {};
    // END STUDENT CODE
  }

  Eigen::Matrix4d quaternionMessageToTransformationMatrix(geometry_msgs::msg::Quaternion quat)
  {
    Eigen::Quaterniond e_quat;
    tf2::fromMsg(quat, e_quat);
    Eigen::Matrix4d result_mat = Eigen::Matrix4d::Identity();
    result_mat.block<3, 3>(0, 0) = e_quat.normalized().toRotationMatrix();
    return result_mat;
  }

  geometry_msgs::msg::Quaternion transformationMatrixToQuaternionMessage(
    Eigen::Matrix4d transform_matrix)
  {
    return tf2::toMsg(Eigen::Quaterniond(transform_matrix.block<3, 3>(0, 0)));
  }
};
}  // namespace coordinate_transform

RCLCPP_COMPONENTS_REGISTER_NODE(coordinate_transform::CoordinateTransformComponent)
