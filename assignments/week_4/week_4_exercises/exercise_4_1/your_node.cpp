#include <ros/ros.h>

// -= (EXERCISE 1) add the include for the string message! =-

// -========================================================-

// -= (EXERCISE 1) your callback function should be defined here! =-

// -===============================================================-

int main(int argc, char** argv){
    ros::init(argc, argv, "your_node");
    ros::NodeHandle p_nh = ros::NodeHandle("~");

    // -= (EXERCISE 1) create the ros subscriber here, and have it use your callback function! =-

    // -========================================================================================-

    ros::spin();
    return 0;
}