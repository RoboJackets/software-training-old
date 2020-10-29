#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <turtle_actionlib/ShapeAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "shape_client");

    // Create an action client connected to /shape_server.
    //
    // Remember to use `true` for the second argument, so it runs in another
    // thread! Otherwise we will hang when we try to wait for the result.


    // Wait until the server boots up


    // Make a goal (turtle_actionlib::ShapeGoal). Fill it out and send it.


    // Wait for the result of the goal.
    // If it is actionlib::SimpleClientGoalState::SUCCEEDED, print out the
    // variables in the result (use rosmsg show to figure out what these are)
    //
    // Otherwise, print out a warning.


    return 0;
}
