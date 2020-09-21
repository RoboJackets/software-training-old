#include <ros/ros.h>

#include <std_msgs/String.h>

// -= (EXERCISE 1) your callback function should be defined here! =-


// -===============================================================-

int main(int argc, char** argv){
    ros::init(argc, argv, "your_node");

    ros::NodeHandle p_nh = ros::NodeHandle("~");

    // -= (EXERCISE 1) create the ros subscriber here, and have it use your callback function! =-

    // -========================================================================================-

    ros::spin();
}