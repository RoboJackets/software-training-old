<!--
STOP
We strongly recommend viewing this file with a rendered markdown viewer. You can do this by:
 - Opening this file in the GitHub web viewer
 - Pressing Ctrl+Shift+V in Visual Studio Code
 - Opening this file in any other markdown viewer you prefer
-->


# Week 1 Project: Coordinate Frame Transformations

## Table of Contents
- [Background](#background)
- [How to Run](#how-to-run)
- [Instructions](#instructions)
  - [Test the simulator](#test-the-simulator)
  - [Implement the rotation matrix helper function](#implement-the-rotation-matrix-helper-function)
  - [Make a vector to hold our transformed tags](#make-a-vector-to-hold-our-transformed-tags)
  - [Write a loop over the old tags](#write-a-loop-over-the-old-tags)
  - [Transform tag position](#transform-tag-position)
  - [Transform tag orientation](#transform_tag_orientation)
  - [Run project](#run-project)

## Background

Explain context and purpose of the exercise

## How to Run

Explain how to launch relevant files. Demonstrate that the robot can't do the desired behavior with the starter code.

## Instructions

Provide the step by step instructions for modifying the code

### Test the simulator
   
Before we start writing code, let's take a moment to introduce you to the robot simulator. To start the simulator, launch the `traini_simulation.launch.py` file in the `traini_bringup` package.

```bash
$ ros2 launch traini_bringup traini_simulation.launch.py
```

**Tip:** Don't forget to source your ROS underlay with `source /opt/ros/foxy/setup.bash`.

You should now see Gazebo with the virtual world we'll be using for our projects.

![Training World](training_world.jpg)

There are two ways to manually drive the robot around. The first uses the `teleop_twist_keyboard` package to drive the robot with your keyboard. To do this, run the `teleop_twist_keyboard` node.

```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Now you should be able to follow the instructions it prints to drive the robot around.

The other way you can drive the robot is using a gamepad, like an XBox controller. To start the joystick control nodes, launch the `joystick_control.launch.py` file.

```bash
$ ros2 launch traini_bringup joystick_control.launch.py
```

You should now be able to drive the robot, using the left joystick to drive forward/backward and the right joystick to turn.

After you've driven your robot around for a bit, select (from the menu bar) `Edit -> Reset Model Poses` to move the robot back to its starting position. This is a good trick to know to reset your simulator.

You can now close the simulator window, or press Ctrl+c in the terminal.

<details>
<summary><b>Bonus:</b> How to fix gamepad mappings</summary>
Different gamepads map their inputs differently. By default, the launch file above uses a config that works with the joysticks we use in the classroom. You can create your own config file to set the mappings appropriate for your gamepad.

To show the content of the default config file, run the following command. You can then copy this to a file anywhere on your computer and edit it there.

<pre><code>$ cat $(ros2 pkg prefix traini_bringup)/share/traini_bringup/config/joystick_parameters.yaml
</code></pre>

Then, you can launch the joystick control nodes with your new config like this:

<pre><code>$ ros2 launch traini_bringup joystick_control.launch.py config_path:=/path/to/your/config/file.yaml
</code></pre>
</details>

### Implement the rotation matrix helper function

Now we'll move on to writing some code. Our first task is to implement the `getRotationMatrixForOpticalFrame()` helper function. Locate this function towards the end of this file:

`~/training_ws/src/software-training/coordinate_transform/src/coordinate_transform.cpp`

**Tip** We'll mark locations in the starter code where you'll need to add code with `// BEGIN STUDENT CODE` and `// END STUDENT CODE`.

This function will create and combine two rotation matrices to create a final transformation matrix that maps from the camera's optical frame to the camera's conventional frame.

Start by declaring and initializing two `std::array` variables, called `R_roll_data` and `R_yaw_data`. Both should contain 16 elements of type `double`.

<details>
<summary><b>Hint:</b> Declaring and initializing a <code>std::array</code></summary>
<p>Here's how you would declare and initialize a <code>std::array</code> that stores three doubles.</p>
<pre><code>std::array&ltdouble, 3&gt my_array = {0, 0, 0};</code></pre>
</details>

These arrays will contain the data for our rotation matrices in what's called "row order". This is a technique for holding a 2-dimensional structure like a matrix in a 1-dimensional container. The array will contain the elements of each row in order from left to right, and top to bottom. So, the following 3x3 matrix:

<table style="border-collapse:collapse">
<tr> <td>1</td> <td>0</td> <td>0</td> </tr>
<tr> <td>0</td> <td>1</td> <td>0</td> </tr>
<tr> <td>0</td> <td>0</td> <td>1</td> </tr>
</table>

Can be represented in row order by this array:

<code>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }</code>

Now, fill out your arrays with the correct values to represent a rotation transformation matrix. `R_roll_data` should represent a rotation of &pi;/2 radians about the X axis. `R_yaw_data` should represent a rotation of &pi;/2 radians about the Z axis.

**Tip:** In C++, you can use the special constant `M_PI` to get the value of &pi;.

<details>
<summary><b>Hint:</b> Rotation matrices</summary>
<p>Rotation about the X axis:</p>
<table style="border-collapse:collapse">
<tr> <td>1</td> <td>0</td> <td>0</td> <td>0</td> </tr>
<tr> <td>0</td> <td>cos(&Theta;)</td> <td>-sin(&Theta;)</td> <td>0</td> </tr>
<tr> <td>0</td> <td>sin(&Theta;)</td> <td>cos(&Theta;)</td> <td>0</td> </tr>
<tr> <td>0</td> <td>0</td> <td>0</td> <td>1</td> </tr>
</table>
<p>Rotation about the Y axis:</p>
<table style="border-collapse:collapse">
<tr> <td>cos(&Theta;)</td> <td>0</td> <td>sin(&Theta;)</td> <td>0</td> </tr>
<tr> <td>0</td> <td>1</td> <td>0</td> <td>0</td> </tr>
<tr> <td>-sin(&Theta;)</td> <td>0</td> <td>cos(&Theta;)</td> <td>0</td> </tr>
<tr> <td>0</td> <td>0</td> <td>0</td> <td>1</td> </tr>
</table>
<p>Rotation about the Z axis:</p>
<table style="border-collapse:collapse">
<tr> <td>cos(&Theta;)</td> <td>-sin(&Theta;)</td> <td>0</td> <td>0</td> </tr>
<tr> <td>sin(&Theta;)</td> <td>cos(&Theta;)</td> <td>0</td> <td>0</td> </tr>
<tr> <td>0</td> <td>0</td> <td>1</td> <td>0</td> </tr>
<tr> <td>0</td> <td>0</td> <td>0</td> <td>1</td> </tr>
</table>
</details>

Next, let's create `Eigen::Matrix4d` objects using our data arrays:

```c++
Eigen::Matrix4d R_roll(R_roll_data.data());
Eigen::Matrix4d R_yaw(R_yaw_data.data());
```

Finally, modify the return statement to return the result of `R_yaw` multiplied by `R_roll`.

### Make a vector to hold our transformed tags

Let's create a `std::vector` to hold our transformed tags. Before we can use `std::vector` in our code, we'll need to include the standard library header that declares it. Locate the student code block at the top of the file, with the rest of the include statements.

Add an include statement for `<vector>`.

Now locate the student code block in `DetectionCallback()` (should be around line 82). Here, declare a `std::vector` called `new_tags` that contains elements of type `stsl_interfaces::msg::Tag`.

A few lines down, you'll see another student code block with this comment:

```c++
// set message tags to new_tags vector
```

We're going to do exactly what the comment says. Add a line that sets `msg`'s `tags` member to our `new_tags` vector.

```c++
new_tag_array_msg.tags = new_tags;
```

### Write a loop over the old tags

Now that we have a container to put our transformed tags into, we need to loop over the container of old tags and, for each one, push a new tag into `new_tags`.

Starting on the line after your `new_tags` declaration, add a for loop that iterates over `tag_array_msg->tags`.

**Tip:** You can get the count of elements in a `std::vector` by calling its `size()` method.

<details>
<summary><b>Hint:</b> For loops</summary>
<p>You can write a for loop that iterates from `i=0` to `i=9` like this:</p>
<pre><code>for(int i = 0; i < 10; ++i)
{
   // body of loop
}</code></pre>
</details>

In the body of this new loop, declare a variable named `new_tag` of type `stsl_interfaces::msg::Tag`. Then, copy the `id` member of the current old tag into `new_tag.id` like this:

```c++
new_tag.id = tag_array_msg->tags[i].id;
```

Finally, at the end of the loop body use `push_back()` to add `new_tag` to the `new_tags` vector.

### Transform tag position

### Transform tag orientation

### Run project

