<!--
STOP
We strongly recommend viewing this file with a rendered markdown viewer. You can do this by:
 - Opening this file in the GitHub web viewer
 - Pressing Ctrl+Shift+V in Visual Studio Code
 - Opening this file in any other markdown viewer you prefer
-->

# Week 7 Project: Linear Quadratic Regulator

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [1 Background](#1-background)
- [2 Running this project](#2-running-this-project)
- [3 Instructions](#3-instructions)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## 1 Background

In this project we will be writing an implementation of Linear Quadratic Regulator.
Really we will be doing something that looks more like an Model Predictive Control application of Linear Quadratic Regulator.
All this means is that have to linearize our nonlinear system around of previous trajectory and we recompute the trajectory every time we get a new state.
You will only be programming some of that linearization and the action client we will be putting into a ROS framework called Nav2.

In the final challenge for this semester, our robot will need to use every other project to complete the entire course.
The key part of that for this week is following a trajectory.
For now we have given you a static trajectory to track using your LQR implementation.

### Nav2

Nav2 is an open source project that contains multiple planners, tracking controllers, and controllers.
The diagram below breaks down the baseline architecture we will be using.

You can see that there is a waypoint to navigate to (our mineral deposit) and a map (our occupancy grid).
The Nav2 architecture is based around things called plugins that implement action servers.
This was chosen because things like planning and tracking a path are long running tasks.
Furthermore, using actions allows the different parts of the nav stack to communicate progress.
The main coordination portion of the architecture is in the behavior tree.
You can think of this as the nav stack deciding when each action should be called.
We have implemented a basic decision tree for you that generates a path to track and then waits until the path tracking action is completed by reaching a goal point.

The static path you are tracking comes from the test_path_generator class.
You do not need to edit this file, everything should work as is.
The lqr_controller.cpp class will be where we implement our LQR controller for path tracking.
You will need to implement the controller_client from scratch.

## 2 Running this project

To run this week's project, you'll need to run one launch file:

```bash
$ ros2 launch rj_training_bringup week_7.launch.xml
```

As usual, this launch file will startup the simulator, rviz, and all of the other nodes the project requires.

By default, this will use the particle filter localizer node you worked on in project 3. You can optionally use the fake localizer with the `use_fake_localizer` argument.
We recommend starting off with the fake localizer and only using the particle filter as a final test to get a feeling for how estimation error can cause issues.

```bash
$ ros2 launch rj_training_bringup week_7.launch.xml use_fake_localizer:=true
```

In rviz you should just see the robot.
It will not move until you implement LQR and the action client.

The second command that won't work until you implement you action client is
```bash
ros2 run lqr_control controller_test_client --ros-args -p use_sim_time:=True
```

This runs the action client and sets use_sim_time to true so that all nodes are using the same time source.
We will come back to this once we have that code finished.

## 3 Instructions

### 3.1 Get the latest starter code

To make sure you're starting with the latest starter code, pull from the git server in your copy of the software-training repository.

```bash
$ cd training_ws/src/software-training
$ git pull
```

You'll also want to make sure you've got the latest version of the training support library by running an apt package update.

```bash
$ sudo apt update
$ sudo apt upgrade
```

If you have done a different installation of stsl that is not through apt make sure to pull the latest code there.

### 3.2 Creating the ControllerTestClient

Create a cpp file in the [lqr_control src folder](../../lqr_control/src) called ControllerTestClient.
This will be a standard ROS2 node and will be in the lqr_control namespace.

<details>
<summary><b>Hint:</b> A standard ROS2 node general form</summary>
<pre><code>

// NAMESPACE is the name of the namespace
// CLASS_NAME is the name of the class

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace NAMESPACE
{

class CLASS_NAME : public rclcpp::Node
{
public:

explicit CLASS_NAME(const rclcpp::NodeOptions & options)
: rclcpp::Node("CLASS_NAME", options)
{
}

} // namespace end for lqr_control
RCLCPP_COMPONENTS_REGISTER_NODE(NAMESPACE::CLASS_NAME)

</code></pre>
</details>

### 3.3 Adding ControllerTestClient to CMakeLists.txt
You will need to add a couple lines to the [CMakeLists.txt](../../lqr_control/CMakeLists.txt) to compile your new node.

The first change is to add your new class as a source to be compiled into the lqr_control library.
This is done by adding the cpp file you created to the add_library call.

```cmake
add_library(lqr_control SHARED
        # BEGIN STUDENT CODE
        src/controller_test_client.cpp
        # END STUDENT CODE
        src/lqr_controller.cpp
        src/test_path_generator.cpp
        )
```

The second thing you will need to do is to register the node as a component.
This uses a specific macro from ROS2 `rclcpp_components_register_node`.

```cmake
# BEGIN STUDENT CODE
rclcpp_components_register_node(
        lqr_control
        PLUGIN "lqr_control::ControllerTestClient"
        EXECUTABLE lqr_controller_test_client
)
# END STUDENT CODE
```

You should be able to run `colcon build` as you normally do without error.

### 3.4 Setting up interface

The next thing we need to do in your ControllerTestClient code is to create an action client that uses the Nav2 FollowPath action type.
You will want to create a using definition at the top of your class as a shorthand.

```c++
using FollowPath = nav2_msgs::action::FollowPath;
```

Next create a class member variable to store the client and initialize the client in the constructor.
Your client should be a shared pointer.


<details>
<summary><b>Hint:</b> Reminder on how to initialize a client</summary>
The client variable syntax is
`rclcpp_action::Client<TYPE>::SharedPtr`.
the syntax to create a client is
`rclcpp_action::create_client<TYPE>(this, "NAME")`
where `TYPE` is the action type.
</details>

<details>
<summary><b>Hint:</b> Written out code</summary>
<pre><code>

explicit ControllerTestClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_test_client", options)
{
client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
}

private:
rclcpp_action::Client<FollowPath>::SharedPtr client_;

</code></pre>
</details>

The next thing you will need to do is create a `nav_msgs::msg::Path` publisher with specific QOS settings.
We want our QOS settings to be `transient_local` because TODO.
You should create this publisher so that is it a shared pointer and a member variable.

### 3.5 Send the goal

Our action client needs to send the target trajectory to the lqr controller.
In order to do this we will implement a method called `SendGoal`.
The method will return nothing and will use the client we just created to send a `FollowPath` action request to the Nav2 action server.
The Nav2 action server will end up calling the plugin we will be creating when we write LQR.
For this step we will just be getting the visualization of the path to show up in rviz.

Write the SendGoal method that
1. Waits for the `/follow_path` action server for up to 10 seconds and prints an error message if it times out.
You can use the following syntax to wait for a response from the action server
```c++
client_->wait_for_action_server(std::chrono::seconds(1))
```
This will wait for 1 second and will return `True` if the action server is ready and false if it times out before getting a response.
2. Uses the TestPathGenerator to create some nominal path
```c++
TestPathGenerator(20).BuildPath();
```
This method returns a `nav_msgs::msg::Path` object.
20 is the number of points to use for the path, leave this at 20.
You will need to set the time to the current ROS time and the frame_id to `/map` to properly see this visualization in rviz.
The path should be stored in an object of type `FollowPath::Goal` in the member called path.
3. publishes the goal path for visualization purposes

Remember that you will want to run the `SendGoal` method in a separate detached thread.
You can create a thread of a member function using this call.
```c++
std::thread(&ControllerTestClient::SendGoal, this).detach();
```

### 3.6 Create response callbacks
Before we call our action we need to create the methods that will be used by the Nav2 stack to communicate back to our client.
All of these methods will just be printing out a message based on the feedback from the Nav2 stack and in one case ending the node.

The first callback we will want to write is the GoalResponseCallback.
This determines if the Nav2 server accepts our target goal.
The method will take in the type `std::shared_future<GoalHandle::SharedPtr>.
The GoalHandle shorthand should be declared at the top using
```c++
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
```
If the goal is accepted the value in the future will be true, otherwise it will be false.
You should use the ROS logger to print out if the server accepted or rejected the goal.
The logger can be called using this syntax,
```c++
RCLCPP_INFO(get_logger(), "MESSAGE");
```

The second callback we will have is the FeedbackCallback.
This is using another Nav2 plugin that is telling you the distance to the goal.
Now we are following a trajectory not trying to reach a specific goal, so this number can increase as we follow the path.
The target point is the last position in the trajectory we are tracking.
The method should have the following parameters,
GoalHandle::SharedPtr,
const std::shared_ptr<const FollowPath::Feedback> feedback)
`
We want the `distance_to_goal` field in the feedback message.
It is of type [`nav2_msgs::action::FollowPath`](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/FollowPath.action).
These fields are filled in by the Nav2 stack.
Print out the distance to the waypoint using the logger.

The third callback is the result callback.
The method takes in a `const GoalHandle::WrappedResult & result`.
Remember this result has 4 different statuses.
Print out a message using the logger to indicate what result code was returned.
Finally shutdown the down using `rclcpp::shutdown()`.

### 3.7 Calling Actions

In this section we will finish the SendGoal function by sending the action using our client.
The first thing we need to do is to create an options variable of type `rclcpp_action::Client<FollowPath>::SendGoalOptions`.
This will be included in our action call and tells the server what callbacks it can call.
To finish off you will need to do the following
1. Set the `goal_response_callback` to the `GoalResponseCallback` method
2. Set the `feedback_callback` to the `FeedbackCallback` method
3. Set the `result_callback` to the `ResultsCallback` method
4. Send the action using the `async_send_goal` method

Remember that in order to bind a member function you will need to use std::bind.
An example call for a method with 1 and 2 variables is below.

```c++
std::bind(&CLASS_NAME::CALLBACK_NAME, this,std::placeholders::_1);
std::bind(&CLASS_NAME::CALLBACK_NAME, this, std::placeholders::_1, std::placeholders::_2);
```

This will finish the action client code.

### 3.8 LQR controller configure function

The code you are looking at is an implementation of a Nav2 controller plugin.
This just means that the Nav2 stack is going to load this class and then call specific functions based on the behavior tree.
The methods that Nav2 calls to setup are configure, then activate.
Everytime we get a new path to track the setPlan method is called.
Everytime we get a new state the computeVelocityCommands is called.
This is where we will call the LQR controller iteratively to compute the next control.

The function you need to implement is the configure function.
In this function you need to
1. save the lifecycle node pointer in a class member.
The type should be `rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr`.
You can do this by just setting a variable of that type equal to the node parameter passed into configure.
2. Pull the following parameters
Note you will have to use the name parameter passed into the node like the following,
`node->declare_parameter<double>(name+".T", 1.0);`.
Change the parameter name (here `T`) to the new name i.e. `node->declare_parameter<std::vector<double>>(name+".Q", {1.0, 1.0, 0.3});`.
   1. T (double): The time horizon of the controller
   2. dt (double): The dt in our discrete dynamics
   3. time_between_states (double): The time it should take the robot to traverse an edge in the trajectory
   4. iterations (int): The number of iterations we should run LQR
   5. Q (std::vector<double>): The diagonal elements of the Q matrix
   6. Qf (std::vector<double>): The diagonal elements of the Qf matrix
   7. R (std::vector<double>): The diagonal elements of the R matrix
4. Ensure the Q,Qf, and R vector parameters are the correct length (3,3,2 respectively).
5. Set the Q,Qf, and R values based on the parameters
6. Allocate the correct size (T/dt) of `prev_x_` (previous states Vector3d), `prev_u_` (previous controls Vector2d), and `S_` (Ricatti Equation Result Matrix3d).
There is a vector constructor that takes in the number of elements as well as the element to put in every location `std::vector<TYPE>(int number, TYPE element)`.
You should be putting a zero value into all `Eigen::TYPE::Zero()` should help.



### 3.9 LQR controller dynamics

Now that we have coded up our action client we will need to implement our LQR controller for path tracking.
The first thing we will need to do is derive the dynamics equations, specifically our A and B matrices.
Our A, B matrices depend on our current state because the differential drive system is nonlinear.
LQR only work with linear A,B matrices, so we are doing a linear approximation using the previous state and control values.
The math for that is shown below

TODO math

A couple hints on Eigen
* `Eigen::Matrix3d` creates a 3x3 matrix, `Eigen::Matrix2d' creates a 2x2 matrix, and `Eigen::Matrix<double, 3, 2>` creates a 3X2 matrix.
* To create an identity matrix use `TYPE::Identity()` where `TYPE` is the types from above.
* To index into an Eigen matrix use `()` not `[]`. So to get the second row and second column use `A(1,1)`
* To multiply the vectors/matrices use `*` like you would normally

<details>
<summary><b>Hint:</b>A Matrix Code Solution</summary>
<pre><code>

Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
A(0, 2) = -u(0)*sin(x(2))*dt_;
A(1, 2) = u(0)*cos(x(2))*dt_;
return A;

</code></pre>
</details>

<details>
<summary><b>Hint:</b>B Matrix Code Solution</summary>
<pre><code>

Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
B(0, 0) = cos(x(2)) * dt_;
B(1, 0) = sin(x(2)) * dt_;
B(2, 1) = dt_;
return B;

</code></pre>
</details>



Finally implement the dynamics equation in the `computeNextState` function.

### 3.10 Commit your new code in git

Once you've got your code for this project working, use the command below to commit it into git. This will make it easier to grab changes to the starter code for the remaining projects.

```bash
$ git commit -a -m "My project 6 code."
```
