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
    cv::Mat input_hsv;
    cv::cvtColor(input, input_hsv, CV_BGR2HSV);
    cv::inRange(input_hsv, range_min, range_max, output);
  }

  void ReprojectToGroundPlane(const cv::Mat &input, const cv::Mat &homography, cv::Mat &output)
  {
    cv::warpPerspective(input, output, homography, input.size() /*TODO this size should be different*/);
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
      cv::namedWindow("obstacles preview", cv::WINDOW_AUTOSIZE);
    }

    ~ObstacleDetectorComponent()
    {
      cv::destroyWindow("obstacles preview");
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

      const auto camera_matrix = cv::Mat(info_msg->k).reshape(1, 3);

      const auto homography = GetHomography(camera_matrix, image_msg->header);

      std::cout << homography << std::endl;

      cv::Mat projected_colors;
      ReprojectToGroundPlane(detected_colors, homography, projected_colors);

      cv::imshow("obstacles preview", projected_colors);
      cv::waitKey(10);

      nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
      occupancy_grid_msg.header.stamp = image_msg->header.stamp;
      occupancy_grid_msg.header.frame_id = "";
      // TODO popuate message
      occupancy_grid_publisher_->publish(occupancy_grid_msg);
    }

    cv::Mat GetHomography(const cv::Mat& camera_matrix, const std_msgs::msg::Header& image_header)
    {
      const auto tf_transform = tf_buffer_.lookupTransform(image_header.frame_id, "base_footprint", image_header.stamp);
      const Eigen::Isometry3d eigen_transform = tf2::transformToEigen(tf_transform);
      cv::Mat opencv_transform;
      cv::eigen2cv(eigen_transform.matrix(), opencv_transform);

      cv::Mat Ra = (cv::Mat_<double>(3,3) << 0, -1, 0, 
                                            -1,  0, 0, 
                                             0, 0, -1);
      cv::Mat Ta = (cv::Mat_<double>(3,1) << 0, 0.75, 1.0);

      cv::Mat Rb = opencv_transform(cv::Range(0,3),cv::Range(0,3)).clone();
      cv::Mat Tb = opencv_transform(cv::Range(0,3),cv::Range(3,4)).clone();

      cv::Mat n = cv::Mat::zeros(3,1,CV_64F);
      n.at<double>(1,0) = -1.0;

      const auto d = Tb.at<double>(1,0);

      cv::Mat M = (Ra*Rb.t()) - (((-Ra*Rb.t()*Tb+Ta)*n.t())/d);

      cv::Mat H = camera_matrix * M * camera_matrix.inv();

      return H;
    }
  };

} // namespace obstacle_detector

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::ObstacleDetectorComponent)
