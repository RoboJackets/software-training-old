# Exercise 4.1
This is the first exercise of week 4, and the first one involving ROS with C++!

# Exercise Objective
For this exercise you will be adding a subscriber to a ROS node. The end result will be a node which
subscribes to `/string_topic`, and prints the messages data to the ROS console.

Adding a subscriber involves:
 
 * Including the correct message header
 * Making a callback function
 * Creating the subscriber object
 
To test that your subscriber works, you will:
 * Run the node using `roslaunch`
 * Find the log file created when the node was running
 * Turn in the node and log file to show your node worked
 
# Message Headers
To use message types, you need to include the correct header.
In this exercise, we will use the `String` message type from the `std_msgs` package.
To include a message, add an include message at the top of the file, like this:
```c++
#include <std_msgs/String.h>
```
Most headers in ROS are included like this, with it usually following the format:
```c++
#include <package_name/Resource.h>
```

# Making the callback function
A callback function should have a `void` return type, and take in a single parameter
with the type of message the subscriber is receiving. 
```c++
void callback_function(std_msgs::String string_msg)
{
    // LOGGING GOES HERE
}
```

To print the string, you need to use `ROS_INFO_STREAM`, which takes a string input
(or something which can easily be converted, like a number type) and prints it to the ROS console. 
It has similar input to `std::cout`, where you can use the `<<` operator to concatenate strings.
There's other variants like `ROS_DEBUG_STREAM` or `ROS_ERROR_STREAM`, but 
for this exercise we just want to display a message, so we use `ROS_INFO_STREAM`. 

Also, be sure to remember that the String message object IS NOT a string object, to 
access the actual string, you need to access it's `data` member variable.

So the final callback function should look something like this:
```c++
void callback_function(std_msgs::String string_msg){
    ROS_INFO_STREAM(string_msg.data);
}
```

# Creating the subscriber object
To create a subscriber, you will use the `subscribe` method of the `NodeHandle` object
already created for you. The type of the object created will to be `ros::Subscriber`.
```c++
ros::Subscriber subscriber = p_nh.subscribe(...);
```

The arguments to this function are:
* The topic we subscribe to, which for this exercise is `"/string_topic"`
* The queue size we want, which should be `1`, because we aren't sending messages so 
fast they can't be processed, and we don't need multiple messages at once.
* A reference to the callback function the subscriber should use. In C++, the way
you reference a function is by using the `&` character right before the name of the 
function, WITHOUT the parenthesis, otherwise the function will be called.

In the end, your subscriber construction should look like this:
```c++
ros::Subscriber subscriber = p_nh.subscribe("/string_topic", 1, &callback_function);
```

# Compile the node and package
To compile the `week_4_exercises` package, we need to put it into a catkin workspace. 
To compile the package, do the following:
* make a copy `week_4_exercises` and put it in your workspace's `src` directory.
* open a terminal and change directory to the workspace
* run `catkin_make`

The package should be compiled! If you run into problems, be sure to double check all your changes.

# Run the node using roslaunch
Now that we've written the node, we need to test it. To do this we'll use roslaunch.
You'll learn more about roslaunch later, but what you need to know right now is that it can
run multiple nodes at the same time.
First, since we've built in a new workspace, we need to tell ros where our files are.
To do this, run this command in a terminal inside `training_ws`
```shell script
$ source devel/setup.bash
```

This lets the terminal you ran the command in find and launch the ros nodes in `training_ws`. 
To launch the node, first you need to run `roscore` in a separate terminal. If you are using
Terminator, use `ctrl+shift+e` or `ctrl+shift+o` to open another terminal instance. 

In one terminal, run 
```shell script
$ roscore
```

In the other, run
```shell script
$ roslaunch week_4_exercises exercise_4_1.launch
```

If you did everything correctly, you should see the following output:
```
[ INFO] [...]: EXERCISE: 1
[ INFO] [...]: wrestling
[ INFO] [...]: navigation
[ INFO] [...]: soccer
[ INFO] [...]: racecar
[ INFO] [...]: computer
[ INFO] [...]: bee
[ INFO] [...]: 
[ INFO] [...]: testing testing 123
```

Use `ctrl+c` to stop the nodes and `roscore`.

# Find the log file created when the node was running
To verify that you completed this assignment, you will need to also turn in the log file
generated from running the node. To find the most recent log file, go to `~/.ros/log/latest`.
There will be several log files, but the one we want is `rosout.log`. 

You can check the contents of the log by using `cat`:
```shell script
$ cat ~/.ros/log/latest/rosout.log
```

# Turn in the node and log file to show your node worked
To verify that you did this exercise, copy the `rosout.log` file into the `exercise_4_1` directory,
then commit your changes and push to github classroom.

To copy the file, you can use the following command:
```shell script
$ cp ~/.ros/log/latest/rosout.log path/to/week_4_exercises/exercise_4_1/
```