#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <turtle_actionlib/ShapeAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "shape_client");

    // Create an action client
    actionlib::SimpleActionClient<turtle_actionlib::ShapeAction> client("/shape_server", true);

    // Wait until the server boots up
    ROS_INFO("Waiting for server");
    client.waitForServer();
    ROS_INFO("Connected!");

    // Make a goal (turtle_actionlib::ShapeGoal). Fill it out and send it.
    turtle_actionlib::ShapeGoal goal;
    goal.edges = 4;
    goal.radius = 0.5;
    client.sendGoal(goal);

    // Wait for the result of the goal.
    // If it is actionlib::SimpleClientGoalState::SUCCEEDED, print out the
    // variables in the result (use rosmsg show to figure out what these are)
    //
    // Otherwise, print out a warning.

    ROS_INFO("Goal sent");
    client.waitForResult();
    ROS_INFO("Done waiting for result");

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Got result with angle %f and apothiem %f",
                 client.getResult()->interior_angle,
                 client.getResult()->apothem);
    } else {
        ROS_WARN("Failed to send a goal");
    }

    return 0;
}
