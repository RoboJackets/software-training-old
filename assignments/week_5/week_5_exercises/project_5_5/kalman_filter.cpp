#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <algorithm>
#include <boost/bind.hpp>

/**
 * A Kalman filter with the following state space:
 * x = [p_x p_y]
 *
 * control inputs:
 * u = [v_x v_y]
 *
 * sensors:
 * y = [gps_x gps_y]
 *
 * Heading is assumed to always be zero.
 */
class KalmanFilter {
public:
    KalmanFilter();
    void gpsCallback(geometry_msgs::PointStamped msg);
    void gpsSensorUpdate(const Eigen::Matrix<double, 2, 1>& y);
    void velocityCallback(geometry_msgs::Twist msg);
    void predict(const Eigen::Matrix<double, 2, 1>& velocity);

private:
    ros::NodeHandle nh;

    // Hardcode to 50Hz for now.
    double dt = 0.02;
    Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d B = dt * Eigen::Matrix2d::Identity();

    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 0.1;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.4;

    Eigen::Vector2d state = Eigen::Vector2d::Zero();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity() * 2.0;

    ros::Subscriber gps_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher pose_pub;
};

KalmanFilter::KalmanFilter() {
    gps_sub = nh.subscribe<geometry_msgs::PointStamped>("/oswin/fix", 1, &KalmanFilter::gpsCallback, this);
    velocity_sub = nh.subscribe<geometry_msgs::Twist>("/oswin/velocity", 1, &KalmanFilter::velocityCallback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/oswin/pose", 1);
}

void KalmanFilter::gpsCallback(geometry_msgs::PointStamped msg) {
    gpsSensorUpdate(Eigen::Vector2d(msg.point.x, msg.point.y));
}

void KalmanFilter::gpsSensorUpdate(const Eigen::Matrix<double, 2, 1>& y) {
    // TODO: Implement the update step!
    // Publish the information as a PoseWithCovariance
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = state(0);
    msg.pose.pose.position.y = state(1);

    std::fill(std::begin(msg.pose.covariance), std::end(msg.pose.covariance), 0);
    msg.pose.covariance[0] = covariance(0, 0);
    msg.pose.covariance[1] = covariance(0, 1);
    msg.pose.covariance[6] = covariance(1, 0);
    msg.pose.covariance[7] = covariance(1, 1);
    pose_pub.publish(msg);
}

void KalmanFilter::velocityCallback(geometry_msgs::Twist msg) {
    // For now hardcode to 50Hz
    predict(Eigen::Vector2d(msg.linear.x, msg.linear.y));
}

void KalmanFilter::predict(const Eigen::Vector2d& u) {
    // TODO: Implement the predict step!
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_filter_node");
    KalmanFilter kf;
    ros::spin();
    return 0;
}
