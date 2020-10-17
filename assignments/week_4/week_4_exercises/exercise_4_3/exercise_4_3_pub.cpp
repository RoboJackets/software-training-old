#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_message_node_pub");
  ros::NodeHandle nh;

  ros::Publisher cos_pub = nh.advertise<std_msgs::Float64>("cosine", 1);
  ros::Publisher sin_pub = nh.advertise<std_msgs::Float64>("sine", 1);

  ros::Rate rate(20);
  while(ros::ok()) {
    ros::Time time = ros::Time::now();

    std_msgs::Float64 cos_msg;
    cos_msg.data = cos(time.toSec());
    cos_pub.publish(cos_msg);
    std_msgs::Float64 sin_msg;
    sin_msg.data = sin(time.toSec());
    sin_pub.publish(sin_msg);

    ros::spinOnce();
    rate.sleep();
  }
}
