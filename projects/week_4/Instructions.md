<!--
STOP
We strongly recommend viewing this file with a rendered markdown viewer. You can do this by:
 - Opening this file in the GitHub web viewer
 - Pressing Ctrl+Shift+V in Visual Studio Code
 - Opening this file in any other markdown viewer you prefer
-->

# Week 4 Project: Optimization

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [1 Background](#1-background)
  - [1.1 The code](#11-the-code)
- [2 Running this project](#2-running-this-project)
- [3 Instructions](#3-instructions)
  - [3.1 Get the latest starter code](#31-get-the-latest-starter-code)
  - [3.2 Inspect /sample_elevation service](#32-inspect-sample_elevation-service)
  - [3.3 Create service client](#33-create-service-client)
  - [3.4 Elevation Sampling: Send SampleElevation request](#34-elevation-sampling-send-sampleelevation-request)
  - [3.5 Elevation Sampling: Wait for SampleElevation response](#35-elevation-sampling-wait-for-sampleelevation-response)
  - [3.6 Hill Climbing: Choose sample positions](#36-hill-climbing-choose-sample-positions)
  - [3.7 Hill Climbing: Sample elevations](#37-hill-climbing-sample-elevations)
  - [3.8 Hill Climbing: Choose goal position](#38-hill-climbing-choose-goal-position)
  - [3.9 Commit your new code in git](#39-commit-your-new-code-in-git)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## 1 Background

This is an exciting week. Our robot will move autonomously for the first time!

This week, we'll be focussing on optimization by doing some literal (simulated) hill climbing. Part of the challenge our robot will need to complete at the end of the semester includes finding the highest spot around it to park. While our real environment is flat, we have simulated terrain data which our robot can measure for the purposes of this task.

If our robot could see a terrain map of the entire world, this task would be really easy. We would just check every spot to find the max elevation and drive directly to it. Unfortunately, we don't have such a map in our scenario. The robot is limitted to checking the elevation at locations within a certain range of it's current position. Because our robot has this limitted view of the world around it, we need to use an iterative optimization algorithm to come up with individual steps that will take us to the peak.

### 1.1 The code

The code we'll be writing this week is in the [peak_finder](../../peak_finder) package. This package features a node that provides a ROS action, `/park_at_peak`, that drives the robot to the highest point and stops. We'll talk in detail about actions later. For now, just know that this is a behavior we can start and cancel.

The bulk of the node has been setup for you. When the action is requested, `PeakFinderComponent::execute()` is called. This is, admittedly, a pretty complicated function since it's juggling a bunch of asynchronous systems. For this project, we only need to understand the high level of what this function is doing. The execute function performs these three steps in a loop:

1. Check the elevation of a few spots around the robot.
1. Pick a spot to drive to.
1. Tell the robot to drive to that location.

These three steps keep repeating until the node determines the robot has reached a local maximum.

There are two things you'll be implementing. The first is the service client used to get our simulated terrain measurements. This will be a client for the `/sample_elevation` service, which checks a location near the robot and replies with the elevation at that location. You'll be setting up the client and implementing the `SampleElevation()` function which uses this client to get the elevation at a specific location. We'll explore more of the details about this service in the steps below.

The second part you'll be implementing is the optimization algorithm used to select the next pose for the robot to move to. This is wrapped up in the `PickNextGoalPosition()` function which takes the robot's current position and elevation and returns the next position the robot should move to. When the robot is at the peak, `PickNextGoalPosition()` should return `current_position` to tell the loop in the `execute()` function to stop.

## 2 Running this project

This week's project is started with a single launch file.

```bash
ros2 launch rj_training_bringup week_4.launch.xml
```

We won't be launching any teleop nodes this week, since our robot will be moving autonomously. You can use the teleop moves to move your robot around, but you'll want to stop any teleop nodes before running the "park at peak" action.

When you launch the project, you'll be greeted by the usual two applications: gazebo and rviz. In rviz, you'll see a new button labelled "Park at Peak". Pressing this button will trigger the park at peak action you'll be implementing in this project. In the rviz 3D viewport, you'll see the simulated terrain map of the world with the highest elevations shown in white and the lowest in black.

The goal is that when you press the "Park at Peak" button, the robot should drive to and stop at the whitest spot on the map.

![GIF of final behavior](working_demo.gif)

Before you write any code, the button will appear to do nothing. The starter code just immediately finishes, reporting that the robot is already at the peak. The code you write will give our robot the behavior we actually want.

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

Checkout into your local branch.
```bash
$ git checkout <your_name>
```

If you have done a different installation of stsl that is not through apt make sure to pull the latest code there.

### 3.2 Inspect /sample_elevation service

Our `peak_finder` node will be using a ROS service, called `/sample_elevation`, to get access to the simulated terrain data. This service lets us request the elevation at a specific location. The location we request must be within 0.15 meters of our robot's current location. To get started on this project, let's get familiar with the `/sample_elevation` service.

Launch the week 4 launch file to get everything running.

In another terminal (with ROS setup scripts sourced), list all of the available service names.

```bash
ros2 service list
```

In that list, you should see `/sample_elevation`. Now, check this service's type with the `type` command.

```bash
ros2 service type /sample_elevation
```

This command will show you the type of the service in the form `package_name/srv/ServiceTypeName`. To see the fields in this type, use the `interface show` command shown below, replacing `<service_type>` with the service type you got from the previous command.

```bash
ros2 interface show <service_type>
```

You should see two sets of fields, separated by `---`. The fields above this line are the request message. The fields below the line are the response message. So, hopefully now it's clear that our client will be sending a location (x and y), and will be getting back the elevation and a success flag. If any of your service requests reply with the success flag set to `false`, you should see an error message logged from the `elevation_server` node.

Try sending an elevation sample request from the command line. You can do this with the `service call` command:

```bash
ros2 service call /sample_elevation stsl_interfaces/srv/SampleElevation '{ x: 0, y: 0 }'
```

Running that command, you should see output like this:

```bash
requester: making request: stsl_interfaces.srv.SampleElevation_Request(x=0.0, y=0.0)

response:
stsl_interfaces.srv.SampleElevation_Response(success=True, elevation=0.6470588235294118)
```

This output is telling us that the success flag is `true` and the elevation at the requested location is about 0.65.

You can now close the week 4 launch session. It's time to start writing some code.

### 3.3 Create service client

Open up [peak_finder_component.cpp](../../peak_finder/src/peak_finder_component.cpp) in the `peak_finder` package. All of our code for this week will be written in this file.

To use our `/sample_elevation` service, we need to setup a service client. For that, we'll need access to the `SampleElevation` service type. Start by including the header file for this service type in the student code comment block at the top of the file.

```C++
#include <stsl_interfaces/srv/sample_elevation.hpp>
```

Now, add a new member for our service client. Find the student code block in the private member variables. Declare a new member variable named `elevation_client_` of type `rclcpp::Client<stsl_interfaces::srv::SampleElevation>::SharedPtr`.

In the `PeakFinderComponent` constructor, find the student code comment block. Initialize `elevation_client_` with a call to `create_client`. The type of the service should be `stsl_interfaces::srv::SampleElevation` and the service name should be `"/sample_elevation"`.

Finally, we need to make sure the service is available before our node starts to execute its action. Find the student code comment block at the start of the `execute` function. In this block, we'll check if the service is "ready". If not, we'll log an error and abort the action goal handle.

```C++
if (!elevation_client_->service_is_ready()) {
  RCLCPP_ERROR(
    get_logger(), "%s service must be available to run peak_finder action!",
    elevation_client_->get_service_name());
  goal_handle->abort(std::make_shared<ParkAtPeak::Result>());
  return;
}
```

Now we can confidently use our client object to call the `/sample_elevation` service while executing the `/park_at_peak` action. 

### 3.4 Elevation Sampling: Send SampleElevation request

Find the student code comment block in the `SampleElevation` function. The `SampleElevation` function is a helper function that will take care of calling the `/sample_elevation` service and waiting for a response. First step, send the request!

To send a request to a service, we need to create and populate a request object. Create a new shared pointer to a `stsl_interfaces::srv::SampleElevation::Request` object. Set the `x` member of the request object to the `x` value of `position`. Do the same for `y`.

**Tip:** To get the x and y values from an `Eigen::Vector2d`, call its `x()` and `y()` member functions.

Finally, call `async_send_request` on the client object, passing in the request object. This function returns a future. Store that future in a variable named `result_future`.

### 3.5 Elevation Sampling: Wait for SampleElevation response

ROS services are asynchronous operations, so the variable we get back from `async_send_request` isn't the response message. It's a future that will hold the response message. We need to wait for that future to be complete and pull the elevation value from it then.

The code in this section picks up right where we left off in the previous section, in the `SampleElevation` function.

We can wait for our future by calling one of its `wait` functions. In this case, we can be confident that if the service hasn't replied in less than 2 seconds, something has gone wrong. So, we'll use the `wait_for` function with a timeout of 2 seconds. If our `wait_for` call hits the timeout, we'll throw an exception to signal the problem up to the code that called `SampleElevation`.

```C++
if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
  throw std::runtime_error("Elevation service call timed out.");
}
```

Here, we use `std::chrono::seconds(2)` to create a [`std::chrono::duration`](https://en.cppreference.com/w/cpp/chrono/duration) object representing 2 seconds. The [chrono library](https://en.cppreference.com/w/cpp/chrono) from the C++ standard gives us several types and functions for working with timepoints and durations.

The `throw` keyword tells the computer to interrupt the normal execution of this function and indicate to the caller that an error has occurred. While we haven't covered exceptions in detail during software training, just know that `throw` marks the end of the function execution, kind of like `return` does. None of the code after the `throw` statement will get executed. If you're interested in learning more about exceptions, take a look at the [exceptions page](https://en.cppreference.com/w/cpp/language/exceptions) on cppreference.com.

If we make it past this check, our future is ready. We can get the response message with the future's `get` function.

```C++
const auto response = result_future.get();
```

If the success flag is set to false, something's gone wrong and we don't have a valid elevation to work with. So, we'll throw an exception.

```C++
if (!response->success) {
  throw std::runtime_error("Elevation server reported failure.");
}
```

And now that we're double, extra, positively certain that our result is valid and ready.... we can return the elevation from the response.

```C++
return response->elevation;
```

With that, `SampleElevation` is done!

### 3.6 Hill Climbing: Choose sample positions

Now we're going to shift our attention to the `PickNextGoalPosition` function. This function implements one iteration our optimization algorithm. Given the robot's current position and elevation, this function needs to return the next position the robot should move to. Once the robot is at the highest spot, this function can signal that upstream by returning `current_position`.

The algorithm we'll implement now is a simple one. It will check a fixed number of locations at a fixed radius around the robot's current location. It will then pick the location with the highest elevation from that set. This isn't the best approach to optimization, but it's easy to implement and smart enough to get the job done.

Our algorithm has two tunable values, so we need to grab two parameters: the radius of our sample circle and the number of samples to take. Find the student code block inside the `PickNextGoalPosition` function and use `get_parameter` to retrieve our tunable values from ROS parameters.

```C++
const double search_radius = get_parameter("search_radius").as_double();
const int sample_count = get_parameter("sample_count").as_int();
```

Now we're going to write the code that generates the sample positions. First, we'll need a container to hold the positions we create. Declare a vector of `Eigen::Vector2d` objects, named `sample_positions`.

Next, create a loop that iterates over angles from 0 to 2*Pi in `sample_count` steps. 

<details>
<summary><b>Hint:</b> Looping over angles.</summary>
<p>To loop around a circle in 10 steps, we could write this:</p>
<pre><code>const auto angle_delta = (2 * M_PI) / 10;
for(auto angle = 0.0; angle < (2 * M_PI); angle += angle_delta) {
}</code></pre>
</details>

In the body of our loop, we'll caclulate the point on the circle for that angle and add it to our vector of sample positions.

```C++
const Eigen::Vector2d pose = (search_radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))) + current_position;
sample_positions.push_back(pose);
```

### 3.7 Hill Climbing: Sample elevations

Now that `sample_positions` holds the set of positions we'll sample, we need to do the sampling. First, declare a vector of doubles called `elevations`. This will hold the elevation values sampled at each position. The idea here is that `elevations[i]` is the elevation at `sample_positions[i]`.

Next, loop through every position in `sample_positions`, call `SampleElevation` with that position, and append the returned value to `elevations`.

### 3.8 Hill Climbing: Choose goal position

Now we need to find the largest elevation and the corresponding sample position. The standard library gives us a function for finding the largest element in a container, called `std::max_element`. This function takes in a first and last iterator, then returns an iterator pointing to the largest value.

```C++
const auto max_elevation_iter = std::max_element(elevations.begin(), elevations.end());
```

Now we can compare this max elevation against the current elevation. If it's less than or equal to our current elevation, we're at the top.

```C++
if (*max_elevation_iter <= current_elevation) {
  return current_position;
}
```

If the max elevation is larger than our current elevation, we need to return the corresponding position. We can do this by getting the index of `max_elevation_iter` with `std::distance` and use that index to get the position from `sample_positions`.

```C++
return sample_positions[std::distance(elevations.begin(), max_elevation_iter)];
```

### 3.9 Commit your new code in git

Once you've got your code for this project working, use the command below to commit it into git. This will make it easier to grab changes to the starter code for the remaining projects.

```bash
$ git commit -a -m "My project 4 code."
```

### 3.10 Trouble shooting

If your code doesn't work, quit the terminal, run `colcon build` at the `training_ws` dir level again, then also make sure you've ran the setup scripts: `source install/setup.bash`. 
