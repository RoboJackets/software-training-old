# Exercise 4.3
Welcome to exercise 4.3. We will be working on writing our own custom ros message

# Exercise Objective
Today we will be looking at writing our own custom message for ROS. We often use
custom messages to combine data into logical chunks. The odometry message you saw in
4.2 was a good example. There we have both the pose (x,y,z orientation) of the robot
and the twist (velocities and angular rates). At the end of the day those are just
different floats in a custom message that someone else wrote. The semantic meaning
of the message is the important part.

# Writing a custom message
I have taken the liberty of doing most of the cmake setup for you. Now, usually you
will see another package used for just messages. This allows people only interact
with your package through ROS. They need those message definitions to send messages
of the correct type, but they do not need to depend on your entire codebase. We
skipped that this week for simplicity.

Now you need to create a .msg file in the msgs/msg directory. You can name your
message anything you would like, you are restricted to the character [a-zA-Z][a-zA-Z1-9_]*
i.e. alphanumeric and _.

## Adding Fields
Now we need to add fields to our message. There a wide range of primitives, you
should only need Float64 (64 is number of bits, don't worry about that for now).
Remember that when creating custom messages in general you can add other messages
to your message. The syntax is

```
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

where fieldtype is a type, and fieldname is the name you would like to use in your code.
Use good names with strong semantic meaning to prevent confusion. You should see something
like this. Remember that you will want to include a `std_msgs/Header` in most
message you create for timestamps and other general information.

```
std_msgs/Header header

std_msgs/Float64 cosine
```

## CMake changes
Now we will need to add out custom message to cmake in order for it to be built.
Go into the CMakeLists.txt at `assignments/week_4/week_4_exercises/msgs/CMakeLists.txt`.
Go to line three
```cmake
add_message_files(
        FILES
        CustomMessage.msg # <===== HERE =======
)

generate_messages(
        DEPENDENCIES
        std_msgs
)
```

Then in the top level for the package `assignments/week_4/week_4_exercises/CMakeLists.txt`
you will need to comment in this line should be line 101
```cmake
#add_subdirectory(msgs) # <======== UNCOMMENT THIS FOR 4.3
```

uncomment means
```cmake
add_subdirectory(msgs) # <======== UNCOMMENT THIS FOR 4.3
```


Add your .msg file name where the `CustomMessage.msg` is. Now compile

### Compile the node and package
To compile the `week_4_exercises` package, we need to put it into a catkin workspace.
To compile the package, do the following:
* make a copy `week_4_exercises` and put it in your workspace's `src` directory.
* open a terminal and change directory to the workspace
* run `catkin_make`

The package should be compiled! If you run into problems, be sure to double check all your changes.

To check that your message is correct, it should be at `catkin_ws/devel/include/week_4_exercises/CustomMessage.h`.


# Adding Your Message
If you look at `exercise_4_3_sub.cpp`  and `exercise_4_3_pub.cpp`. There are two simple
nodes, one that publishes two topics and one that subscribes to two topics. Now
we can see that in exercise_4_3_sub.cpp we use both messages from the different callbacks
to publish a single message. It is a little weird to use two topics to send data
that can be grouped together. Your goal is to create a custom message that has two Float64
and a header. You will then modify the code to use that custom message instead of
the two topics. You will also need to change the subscriber node to subscribe to
a topic with the message type of your custom message. There are some key reminders below


## Compile the node and package
To compile the `week_4_exercises` package, we need to put it into a catkin workspace.
To compile the package, do the following:
* make a copy `week_4_exercises` and put it in your workspace's `src` directory.
* open a terminal and change directory to the workspace
* run `catkin_make`

The package should be compiled! If you run into problems, be sure to double check all your changes.

To check that your message is correct, it should be at `catkin_ws/devel/include/week_4_exercises/CustomMessage.h`.

## Message Includes
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

Even the header you write will be of the similar format for the include. For example
our package is named `week_4_exercises` and our custom message was `CustomMessage.msg`
our include would be
```c++
#include <week_4_exercises/CustomMessage.h>
```


## Run the nodes using rosrun
Now that we've written the nodes, we need to test it.
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
$ rosrun week_4_exercises custom_message_node_sub
```

In one more, run
```shell script
$ rosrun week_4_exercises custom_message_node_pub
```

