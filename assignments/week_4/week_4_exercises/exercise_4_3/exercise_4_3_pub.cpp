#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <week_4_exercises/CustomMessage.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_message_node_pub");
  ros::NodeHandle nh;

  ros::Publisher cos_pub = nh.advertise<std_msgs::Float64>("cosine", 1);
  ros::Publisher sin_pub = nh.advertise<std_msgs::Float64>("sine", 1);

  ros::Publisher custom_msg_pub = nh.advertise<week_4_exercises::CustomMessage>("cosine_sine", 1);

  ros::Rate rate(20);
  while(ros::ok()) {
    ros::Time time = ros::Time::now();

    std_msgs::Float64 cos_msg;
    cos_msg.data = cos(time.toSec());
    cos_pub.publish(cos_msg);
    std_msgs::Float64 sin_msg;
    sin_msg.data = sin(time.toSec());
    sin_pub.publish(sin_msg);

    week_4_exercises::CustomMessage msg;
    msg.header.stamp = ros::Time::now();
    msg.cosine = cos_msg;
    msg.sine = sin_msg;
    custom_msg_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
