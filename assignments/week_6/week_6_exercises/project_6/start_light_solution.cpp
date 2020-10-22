#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <math.h>

ros::Publisher debug_img_pub;
ros::Publisher start_pub;

std_msgs::Bool prev_start_msg;
ros::Time last_red_time;

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

bool colorOn(cv::Mat color_img) {
    std::vector<std::vector<cv::Point>> contours;
    findContours(color_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour, false);
        double circularity = 4 * M_PI * (area / (perimeter * perimeter));

        if (.6 < circularity && circularity < 1.2 && area > 100)  // These could be launch params
            return true;
    }
    return false;
}

void img_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    // Uncomment to keep publushing true, if green previously seen
//    if (prev_start_msg.data) {
//        start_pub.publish(prev_start_msg);
//        return;
//    }

    cv::Mat hsv_frame, red_found, green_found;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_found);
    cv::inRange(hsv_frame, cv::Scalar(20, 120, 120), cv::Scalar(100, 255, 255), green_found);

    cv::dilate(green_found, green_found, kernel(2, 2));
    cv::dilate(red_found, red_found, kernel(2, 2));

    bool red_on = colorOn(red_found);
    bool green_on = colorOn(green_found);

    if (red_on)
        last_red_time = msg->header.stamp;

    prev_start_msg.data = green_on && (msg->header.stamp - last_red_time).toSec() < 1;
    start_pub.publish(prev_start_msg);

    sensor_msgs::Image outmsg;
    cv_ptr->image = red_found;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);
    debug_img_pub.publish(outmsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "start_light");
    ros::NodeHandle pnh{"~"};

    ros::Subscriber img_sub = pnh.subscribe("/camera/image", 1, img_callback);

    debug_img_pub = pnh.advertise<sensor_msgs::Image>("debug_img", 1);
    start_pub = pnh.advertise<std_msgs::Bool>("/event/race_started", 1);

    ros::spin();
}