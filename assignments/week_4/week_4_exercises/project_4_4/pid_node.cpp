#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher velocity_pub_; // Publisher for velocity
ros::Publisher error_pub_;    // Publisher for error

geometry_msgs::PoseStamped kyle_pose_; // Pose of top turtle
ros::Time last_msg_time_;              // Last time callback was called (to calculate delta t)

double integral_, pre_error_;

// PID Global Variables
double Kp_ = 1;
double Ki_ = 0;
double Kd_ = 0;

/**
 * Callback for Kyle (top turtle). Saves the position of the top turtle into the global
 * variable kyle_pose_
 * @param msg message containing Kyle's pose
 */
void kylePoseCallback(geometry_msgs::PoseStamped msg)
{
    // IMPLEMENT!
}


/**
 * Uses PID terms (Kp, Ki, Kd) to comput output given error and delta time
 * @param error error feed into PID (goal.x - current.x)
 * @param dt delta time from last update to this one
 */
double pid(double error, double dt) {
    // IMPLEMENT!

    // Proportional term

    // Integral term

    // Derivative term

    // Calculate total output

    return 0;
}


/**
 * Callback for Oswin (bottom turtle). Calculates the error in x between the two turtles,
 * and then uses a PID Controller to calculate a control to publish
 * @param msg message containing Oswin's pose
 */
void oswinPoseCallback(geometry_msgs::PoseStamped msg)
{
    // IMPLEMENT!

    // Check if last_msg_time_ is populated (not first callback call)

    // Calculate error in x between top and bottom turtles and the delta time

    // Call PID function to get controls

    // Save message time in last_msg_time_

    // publish a geometry_msgs::Twist message so that the turtle will move

    // publish a std_msgs::Float64 message to be able to graph the error in rqt_plot
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");

    // IMPLEMENT!

    // Create global nodehandle

    // Advertise "/oswin/velocity" to control the bottom turtle and "/error" for visualization

    // Subscriber to both ground truth topics to get their positions

    // Don't forget to call ros::spin() to let ros do things behind the scenes and call your callback
    // functions when it receives a new message!
}
