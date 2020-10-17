//
// Created by jason on 10/17/20.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <random>

ros::Publisher gps_pub;
std::default_random_engine generator;
std::normal_distribution<double> distribution;

void poseCallback(const geometry_msgs::PoseStampedConstPtr msg) {
  geometry_msgs::PointStamped point;
  point.header = msg->header;
  point.point.x = msg->pose.position.x + distribution(generator);
  point.point.y = msg->pose.position.y + distribution(generator);
  point.point.z = 0;

  gps_pub.publish(point);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "buzzsim_helper");
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pNh = ros::NodeHandle("~");

  srand (100);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/oswin/velocity", 1);
  gps_pub = nh.advertise<geometry_msgs::PointStamped>("/oswin/fix", 1);
  ros::Subscriber pose_sub = nh.subscribe("/oswin/ground_truth", 1, poseCallback);

  double mean, variance;
  pNh.param("mean", mean, 0.0);
  pNh.param("variance", variance, 1.0);
  distribution =std::normal_distribution<double>(mean, variance);

  ros::Duration wait(5);
  wait.sleep();

  geometry_msgs::Twist msg;

  ros::Rate rate(20);
  while(ros::ok()) {
    ros::Time time = ros::Time::now();

    msg.linear.x = 0.5;
    msg.angular.z = 0.1;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
