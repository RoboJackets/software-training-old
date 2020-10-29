# Exercise 6.2: Writing an action client
By now, you've seen a little bit about how ROS actions work. Let's put that theory into practice by writing a client for an action!

We've written an 


## ROS Actions
Remember that ROS actions are just groupings of a few key topics:
 - Goal
 - Feedback
 - Result

Actions are intended for long-running things that your robot might do. In this exercise, we'll use the `turtlesim` node that comes built-into ROS (the original inspiration for the Buzzsim simulator you've seen). Specifically, turtlesim has a built-in action for drawing a shape with the turtle by driving it around over the screen.

There are two parts required to run an action: the action _server_ and the action _client_. The client initiates requests (i.e. "draw a new shape"), and the server responds to these requests, running some computation or making something happen in the world (i.e. making the turtle move in a triangle).

You can test this out by running the following nodes:
 - `rosrun turtlesim turtlesim_node`
 - `rosrun turtle_actionlib shape_server`

> Note: don't forget to run `roscore`!

Then, we can publish a message:
```sh
rostopic pub /turtle_shape/goal turtle_actionlib/ShapeActionGoal "$(cat path/to/week_6/test_goal.txt)
```

You should see the turtle draw a triangle. If you're curious, you can look at `test_goal.txt`, and change its values to alter the value of the goal.

Note that we are not looking at feedback or result messages. If you want, you can do that now using `rostopic echo` - they should be on `/turtle_shape/feedback` and `turtle_shape/result`.

## Making an Action Client
Now, let's do this programmatically from C++ code. Recall that we can make an action client as so:
```cpp
actionlib::SimpleActionClient<package_name::ActionName> action_client("/path/to/action", true);
```

> Note: This can be found in `<actionlib/client/simple_action_client.h>`.

> Note: The turtlesim messages for drawing a shape are in `<turtle_actionlib/ShapeAction.h>

Next, you'll want to wait for the server to start up if it hasn't already. You can call the `waitForServer()` method on your action client to block until this occurs.

There are a few other methods from `SimpleActionClient` you'll find useful:
 - `.sendGoal(package_name::GoalName goal)`
 - `.waitForResult(ros::Duration timeout)` - returns `true` if the action completed or was cancelled, and false if it timed out
 - `.getState()` - gets the action's state (i.e. `PENDING`, `ACTIVE`, `SUCCEEDED`, `ABORTED`, etc). You can get this object as a string by calling `.toString()` on it.

Using these methods, create an action client for the `turtle_actionlib::ShapeAction` type, and send a goal message to it with some number of sides and some defined radius. Then run your client with the `exercise_6_2.launch` launch file, and verify that the turtle drives in a polygon.
