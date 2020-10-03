#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc, char** argv){
    ros::init(argc, argv, "testing_node");

    ros::NodeHandle p_nh = ros::NodeHandle("~");
    ros::Publisher publisher = p_nh.advertise<std_msgs::String>("/string_topic", 1);

    int exercise;
    p_nh.getParam("/testing_node/exercise", exercise);

    ROS_INFO_STREAM("EXERCISE: " << exercise);

    std::string test_strings[] = {
            "robotics",
            "wrestling",
            "navigation",
            "soccer",
            "racecar",
            "computer",
            "bee",
            "",
            "testing testing 123"
    };

    switch (exercise) {
        case 1:
            // exercise 1
            for(const std::string& str : test_strings){
                std_msgs::String msg;
                msg.data = str;
                publisher.publish(msg);
                ros::Duration(0.5).sleep();
            }
            break;
        case 2:
            // exercise 2
            break;
        case 3:
            // exercise 3
            break;
    }
    return 0;
}