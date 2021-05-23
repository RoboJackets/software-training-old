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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core/eigen.hpp>

namespace obstacle_detector
{
  void FindColors(const cv::Mat &input, const cv::Scalar &range_min, const cv::Scalar &range_max, cv::Mat &output)
  {
    //// BEGIN STUDENT CODE
    cv::Mat input_hsv;
    cv::cvtColor(input, input_hsv, CV_BGR2HSV);
    cv::inRange(input_hsv, range_min, range_max, output);
    //// END STUDENT CODE
  }

  void ReprojectToGroundPlane(const cv::Mat &input, const cv::Mat &homography, const cv::Size& map_size, cv::Mat &output)
  {
    //// BEGIN STUDENT CODE
    cv::warpPerspective(input, output, homography, map_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(127));
    //// END STUDENT CODE
  }

  class ObstacleDetectorComponent : public rclcpp::Node
  {
  public:
    explicit ObstacleDetectorComponent(const rclcpp::NodeOptions &options)
        : rclcpp::Node("obstacle_detector", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
      camera_subscriber_ = image_transport::create_camera_subscription(
          this, "/camera/image_raw",
          std::bind(&ObstacleDetectorComponent::ImageCallback, this, std::placeholders::_1,
                    std::placeholders::_2),
          "raw", rmw_qos_profile_sensor_data);
      occupancy_grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid", 1);
      declare_parameters<int>("obstacle_color_range", {{"min.h", 0},
                                                       {"min.s", 0},
                                                       {"min.v", 0},
                                                       {"max.h", 0},
                                                       {"max.s", 0},
                                                       {"max.v", 0}});
      declare_parameter<double>("map_resolution", 100.0);  // px / m
      declare_parameter<int>("map_width", 50);  // px
      declare_parameter<int>("map_height", 100);  // px
    }

  private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    image_transport::CameraSubscriber camera_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
      const auto cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
      const auto min_color = cv::Scalar(get_parameter("obstacle_color_range.min.h").as_int(),
                                        get_parameter("obstacle_color_range.min.s").as_int(),
                                        get_parameter("obstacle_color_range.min.v").as_int());
      const auto max_color = cv::Scalar(get_parameter("obstacle_color_range.max.h").as_int(),
                                        get_parameter("obstacle_color_range.max.s").as_int(),
                                        get_parameter("obstacle_color_range.max.v").as_int());

      cv::Mat detected_colors;
      FindColors(cv_image->image, min_color, max_color, detected_colors);

      std::string tf_error_string;
      if (!tf_buffer_.canTransform("base_footprint", image_msg->header.frame_id, image_msg->header.stamp, tf2::durationFromSec(0.1), &tf_error_string))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not lookup transform. %s", tf_error_string);
        return;
      }

      const auto map_resolution = get_parameter("map_resolution").as_double();
      const auto map_size = cv::Size(get_parameter("map_width").as_int(), get_parameter("map_height").as_int());
      cv::Mat map_camera_intrinsics;
      cv::Mat map_camera_rotation;
      cv::Mat map_camera_position;
      GetMapCameraProperties(map_resolution, map_size, map_camera_intrinsics, map_camera_rotation, map_camera_position);

      const auto camera_matrix = cv::Mat(info_msg->k).reshape(1, 3);

      const auto homography = GetHomography(camera_matrix, image_msg->header, map_camera_intrinsics, map_camera_rotation, map_camera_position);

      cv::Mat projected_colors;
      ReprojectToGroundPlane(detected_colors, homography, map_size, projected_colors);

      cv::flip(projected_colors, projected_colors, 0);

      nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
      occupancy_grid_msg.header.stamp = image_msg->header.stamp;
      occupancy_grid_msg.header.frame_id = "base_footprint";
      occupancy_grid_msg.info.height = map_size.height;
      occupancy_grid_msg.info.width = map_size.width;
      occupancy_grid_msg.info.resolution = 1.0/map_resolution;
      occupancy_grid_msg.info.map_load_time = image_msg->header.stamp;
      occupancy_grid_msg.info.origin.position.x = 0;
      occupancy_grid_msg.info.origin.position.y = map_size.width/(2.0 * map_resolution);
      occupancy_grid_msg.info.origin.position.z = 0;
      occupancy_grid_msg.info.origin.orientation.x = 0;
      occupancy_grid_msg.info.origin.orientation.y = 0;
      occupancy_grid_msg.info.origin.orientation.z = -0.7071068;
      occupancy_grid_msg.info.origin.orientation.w = 0.7071068;
      std::transform(projected_colors.begin<uint8_t>(), projected_colors.end<uint8_t>(), std::back_inserter(occupancy_grid_msg.data), [](const auto& pixel_value){
        switch(pixel_value) {
          case 0:
            return 0;
          case 255:
            return 100;
          case 127:
          default:
            return -1;
        }
      });

      occupancy_grid_publisher_->publish(occupancy_grid_msg);
    }

    void GetMapCameraProperties(const double map_resolution, const cv::Size& map_size, cv::Mat& intrinsics, cv::Mat& rotation, cv::Mat& position)
    {
      rotation = (cv::Mat_<double>(3,3) << 0, -1,  0, 
                                          -1,  0,  0, 
                                           0,  0, -1);
      const auto z = 1.0;
      const auto f = map_resolution * z;
      const auto y = map_size.height / (2 * map_resolution);
      position = (cv::Mat_<double>(3,1) << 0, y, z);
      intrinsics = (cv::Mat_<double>(3,3) << f, 0, (map_size.width/2.0), 0, f, (map_size.height/2.0), 0, 0, 1);
    }

    cv::Mat GetHomography(const cv::Mat& camera_intrinsics, const std_msgs::msg::Header& image_header, const cv::Mat& map_camera_intrinsics, const cv::Mat& map_camera_rotation, const cv::Mat& map_camera_position)
    {
      const auto tf_transform = tf_buffer_.lookupTransform(image_header.frame_id, "base_footprint", image_header.stamp);
      const Eigen::Isometry3d eigen_transform = tf2::transformToEigen(tf_transform);
      cv::Mat opencv_transform;
      cv::eigen2cv(eigen_transform.matrix(), opencv_transform);

      cv::Mat Rb = opencv_transform(cv::Range(0,3),cv::Range(0,3)).clone();
      cv::Mat Tb = opencv_transform(cv::Range(0,3),cv::Range(3,4)).clone();

      // Assumes ground plane lies at Z=0 with a normal pointing towards +Z, in the base_footprint frame
      cv::Mat n = Rb * (cv::Mat_<double>(3,1) << 0,0,1);
      const auto d = cv::norm(Tb.mul(n));

      cv::Mat M = (map_camera_rotation*Rb.t()) - (((-map_camera_rotation*Rb.t()*Tb+map_camera_position)*n.t())/d);

      cv::Mat H = map_camera_intrinsics * M * camera_intrinsics.inv();

      return H;
    }
  };

} // namespace obstacle_detector

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::ObstacleDetectorComponent)
