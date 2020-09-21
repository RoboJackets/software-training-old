
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define STR_INDIR(x) #x
#define STR(x) STR_INDIR(x)
//#define CURRENT_DIR CUR_DIR


int main(int argc, char** argv) {
  ros::init(argc, argv, "rqt_plotter_node");
  ros::NodeHandle nh;

  ros::Publisher cos_pub = nh.advertise<std_msgs::Float64>("cosine", 10);
  ros::Publisher sin_pub = nh.advertise<std_msgs::Float64>("sine", 10);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/robo_buzz", 1);

  std::string file_path = STR(CUR_DIR);
  std::cout << "load at file path " << file_path << std::endl;
  cv::Mat img = imread(file_path+"/RoboBuzz.png", cv::IMREAD_COLOR);
  if(img.empty()) {
    std::cerr << "failed to load image, faulting" << std::endl;
    exit(0);
  }

  ros::Rate rate(10);
  while(ros::ok()) {
    ros::Time time = ros::Time::now();

    std_msgs::Float64 cos_msg;
    cos_msg.data = cos(time.toSec());
    cos_pub.publish(cos_msg);
    std_msgs::Float64 sin_msg;
    sin_msg.data = sin(time.toSec());
    sin_pub.publish(sin_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = cos_msg.data;
    odom_msg.pose.pose.position.y = sin_msg.data;

    odom_msg.twist.twist.angular.x = cos_msg.data;
    odom_msg.twist.twist.angular.y = sin_msg.data;
    odom_pub.publish(odom_msg);

    // publish sample image that moves over time
    float angle = cos(time.toSec()) * 180 / M_PI;
    cv::Point2f center((img.cols-1)/2.0, (img.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), img.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - img.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - img.rows/2.0;

    cv::Mat dst = cv::Mat(img);
    cv::warpAffine(img, dst, rot, bbox.size());

    cv_bridge::CvImage cv_image;
    cv_image.image = dst;
    cv_image.encoding = "bgr8";
    sensor_msgs::Image image_msg;
    cv_image.toImageMsg(image_msg);
    img_pub.publish(cv_image);


    ros::spinOnce();
    rate.sleep();
  }
}
