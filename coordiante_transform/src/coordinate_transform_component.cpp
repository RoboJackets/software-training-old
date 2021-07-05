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
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace std::chrono_literals;

namespace coordinate_transform {
class CoordinateTransformComponent : public rclcpp::Node
{
public:
  explicit CoordinateTransformComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("coordinate_transformer", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    tag_sub_ = this->create_subscription<stsl_interfaces::msg::TagArray>("/aruco_tag_detector/tags",
            10, std::bind(&CoordinateTransformComponent::DetectionCallback, this, std::placeholders::_1));

    tag_publisher_ = this->create_publisher<stsl_interfaces::msg::TagArray>("~/tags_transformed", 1);
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/tags_visual", 1);
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  rclcpp::Publisher<stsl_interfaces::msg::TagArray>::SharedPtr tag_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

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
    const Eigen::Isometry3d eigen_transform = tf2::transformToEigen(tf_transform);

    // creates a matrix that goes from camera to standard ROS coordinates
    Eigen::Matrix4d optical = getRotMatForOpticalFrame();

    // BEGIN STUDENT CODE
    // create a new tag array message
    stsl_interfaces::msg::TagArray new_tag_array_msg;
    // copy the header information
    new_tag_array_msg.header = tag_array_msg->header;
    // change the frame_id to be the correct reference frame
    new_tag_array_msg.header.frame_id = "base_footprint";
    // END STUDENT CODE

    // BEGIN STUDENT CODE
    // iterate over each tag to and transform it into body frame
    for(int i = 0; i < tag_array_msg->tags.size(); i++) {
      stsl_interfaces::msg::Tag new_tag;
      new_tag.id = tag_array_msg->tags[i].id;

      geometry_msgs::msg::Point old_tag_position = tag_array_msg->tags[i].pose.position;

      // extract the important parts of the pose
      Eigen::Vector4d pose = Eigen::Vector4d(old_tag_position.x, old_tag_position.y, old_tag_position.z, 1);

      // apply the transform
      pose = eigen_transform.matrix() * optical * pose;

      new_tag.pose.position.x = pose(0);
      new_tag.pose.position.y = pose(1);
      new_tag.pose.position.z = pose(2);

      // get the orientation of the tag
      Eigen::Matrix4d tag_orientation = convertQuatToRotMat(tag_array_msg->tags[i].pose.orientation);
      tag_orientation = eigen_transform.matrix() * optical * tag_orientation;
      new_tag.pose.orientation = convertRotMatToQuat(tag_orientation);

      new_tag_array_msg.tags.push_back(new_tag);
    }
    tag_publisher_->publish(new_tag_array_msg);
    // END STUDENT CODE

    // visualization code
    visualization_msgs::msg::MarkerArray marker_array;
    for(stsl_interfaces::msg::Tag tag : new_tag_array_msg.tags)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_footprint";
      marker.header.stamp = tag_array_msg->header.stamp;
      marker.id = tag.id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(0.25s);

      marker.pose.orientation.x = tag.pose.orientation.x;
      marker.pose.orientation.y = tag.pose.orientation.y;
      marker.pose.orientation.z = tag.pose.orientation.z;
      marker.pose.orientation.w = tag.pose.orientation.w;

      marker.pose.position.x = tag.pose.position.x;
      marker.pose.position.y = tag.pose.position.y;
      marker.pose.position.z = tag.pose.position.z;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.025;

      marker.color.a = 0.9;
      marker.color.r = 0.0;
      marker.color.b = 1.0;
      marker.color.g = 0.0;

      marker_array.markers.push_back(marker);
    }
    visualization_pub_->publish(marker_array);
  }

  Eigen::Matrix4d getRotMatForOpticalFrame() {
    // BEGIN STUDENT CODE
    Eigen::Matrix4d R_roll;
    R_roll << 1, 0, 0, 0,
            0, cos(M_PI/2), sin(M_PI/2), 0,
            0, -sin(M_PI/2), cos(M_PI/2), 0,
            0,0,0,1;

    Eigen::Matrix4d R_yaw;
    R_yaw << cos(M_PI/2), sin(M_PI/2), 0, 0,
            -sin(M_PI/2), cos(M_PI/2), 0, 0,
            0, 0, 1, 0,
            0,0,0,1;
    return R_yaw * R_roll;
    // END STUDENT CODE
  }

  Eigen::Matrix4d convertQuatToRotMat(geometry_msgs::msg::Quaternion quat) {
    Eigen::Quaterniond e_quat = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    Eigen::Matrix4d result_mat = Eigen::Matrix4d::Zero();
    result_mat.block<3,3>(0,0) = e_quat.normalized().toRotationMatrix();
    result_mat(3,3) = 1.0;
    return result_mat;
  }

  geometry_msgs::msg::Quaternion convertRotMatToQuat(Eigen::Matrix4d homo_mat) {
    geometry_msgs::msg::Quaternion quat;
    Eigen::Quaterniond e_quat = Eigen::Quaterniond(homo_mat.block<3,3>(0,0));

    quat.x = e_quat.x();
    quat.y = e_quat.y();
    quat.z = e_quat.z();
    quat.w = e_quat.w();

    return quat;
  }
};
} // namespace coordinate_transform

RCLCPP_COMPONENTS_REGISTER_NODE(coordinate_transform::CoordinateTransformComponent)
