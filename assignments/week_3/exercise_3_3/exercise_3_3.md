# Exercise 3.3
Welcome to your first ROS exercise! We will first give a brief overview of what ROS is,
then we will learn the basics of ROS nodes and learn how to launch nodes, and some commandline tools.

## Baclground on ROS
The core of ROS deals with multiple small programs, called __nodes__, which communicate
with each other. The important thing is that each node is only responsible for one aspect
of the robot, which helps make the entire stack much cleaner.

For example, we might have a
nodes for identifying barrels, mapping, localization (finding out where the robot is) and
path planning.

## ROS Nodes and Launching Buzzsim
Before we write our own node, let's first launch an existing node. Because the ROS ecosystem
is quite large, we can easily use ROS nodes which other people have written for our own projects.

First you'll need to build the simulator. Go to the Robojackets/Buzzsim/catkin_ws folder in your VM using a terminal. Then type catkin_make and hit enter. From a new terminal you can also type these commands in order:

```bash
cd RoboJackets/Buzzsim/catkin_ws
catkin_make
```
This will build buzzsim, a simulation which we will use for many ROS exercises.

Then we need to run 'roscore', still in catkin_ws:
```bash
roscore
```
Without launching `roscore` first, you'll see an error along the lines of:
```
[ERROR] [1567021515.828919061]: [registerPublisher] Failed to contact master at [localhost:11311].  Retrying...
```

Let's start by launching the `buzzsim` simulator that we will make use of the coming few weeks. Run this in another
terminal window, once again within the same catkin_ws folder:
```bash
rosrun buzzsim buzzsim
```
Here, the second `buzzsim` is the name of the **executable**, while the first `buzzsim` is the name of the
**ROS package** which the `buzzsim` executable is from.

You can think of a **ROS package** as an organizational unit under which we can put ROS nodes,
libraries, scripts and more.

If you get the error:
```
[rospack] Error: package 'buzzsim' not found
```

then this means that you have not sourced the file at `catkin_ws/devel/setup.bash` (or `catkin_ws/devel/setup.zsh` if
you're using zsh):
```bash
cd catkin_ws # Or wherever your catkin_ws is located
source devel/setup.bash
```

After launching the node, you should see the simulator window open up:

![buzzsim](https://github.com/RoboJackets/ros-training/blob/master/code/instructions/buzzsim.png)

## ROS Topics and Keyboard Teleop
Now that we've got the simulator up and running, let's try moving the turtle around. To do that, we
need to communicate to the simulator node. ROS does this through the use of **topics** and **messages**.

You can think of a ROS topic as an address that messages are sent to, and ROS messages as the actual messages
that are sent to those topics.

For example, the simulator node listens to messages on the `/oswin/velocity` topic, makes the turtle move
depending on what message was sent. This means that we can control the turtle by sending a message on the
`/oswin/velocity` topic.

<img src="https://drive.google.com/uc?export=download&id=1eMJvdkT4IcLWoMIsSyF7P_bzkNNJW8yU">

Let's try this. Open up another terminal window, and first install 'teleop_twist_keyboard' with the following command:

```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

Then launch the `teleop_twist_keyboard` node like so:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

Ignore the details of the syntax for now, but this launches the `teleop_twist_keyboard` node from the
`teleop_twist_keyboard` package. You should see something like this in the terminal window:

![keyboard_teleop](https://github.com/RoboJackets/ros-training/blob/master/code/instructions/keyboard_teleop.png)

Try pressing the keys in the image. You should see the turtle in the simulator start to move around.

What's happening is that the `teleop_twist_keyboard` node **publishes** the velocity commands to the
`/oswin/velocity` topic and because `buzzsim` node **subscribes** to the same topic, it is able to
receive those messages and move the turtle. We call the `teleop_twist_keyboard` a **ROS Publisher**
and the `buzzsim` node a **ROS Subscriber**.

<img src="https://drive.google.com/uc?export=download&id=1IMTA2zjoZJ-V2lv87u_clImAZ7xkhvEK">

## The `rostopic` tool
To look more into this, we can use the `rostopic` tool. Close the `teleop_twist_keyboard` node with
Ctrl-C, and type the following:
```bash
rostopic --help
```

The help screen of `rostopic` should show, listing out the different commands that are possible with
`rostopic`. Let's try out the `list` options:
```bash
rostopic list
```

You should see a few topics:
```
/oswin/velocity
/rosout
/rosout_agg
/tf
```

Notice that the `/oswin/velocity` topic is shown here. We can get more information about a ROS topic
with the `info` command:
```
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /buzzsimsim (http://{YOUR USER NAME HERE}:36899/)
```

Here, we can see that this topic is of type `geometry_msgs/Twist`, has no `Publishers`, and has one `Subscriber`,
which is the `buzzsim` node. The type of a topic is the type of message of that topic. In this case, it is
`geometry_msgs/Twist`, which contains information about the velocity of the robot.

Since `geometry_msgs/Twist`, which comes from the package `geometry_msgs` package from ROS, we can
lookup the definition in the [ROS docs](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
(Google "geometry_msgs/Twist" and it should show up)

<img src="https://i.imgur.com/uovZmPe.png">

### `rostopic echo`
We can see exactly what is being sent on a topic with the `echo` command:
```bash
rostopic echo /oswin/velocity
```

You should see nothing being printed out: That's because nothing is being published to the this topic right now.
Let's publish something to this topic. Open up another terminal window, and open up the `teleop_twist_keyboard` node
again: 
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

This time when you move the turtle around using the keyboard teleop, you should see the exact messages being published
in the terminal window with `rostopic echo`, something like:
```
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

### `rostopic pub`
We can also publish the messages directly from the command line also, using the `pub` command. Close the `teleop_twist_keyboard`
node again, and then run the following:
```bash
rostopic pub /oswin/velocity geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Note that tab completion is really useful here. Type `rostopic pub /oswin/velocity ` and then keep tabbing. Also,
the identation matters (for those of you who know YAML, this is YAML).

The `rostopic pub` command will publish a message of type `geometry_msgs/Twist` to the `/oswin/velocity` topic,
and you should now see the turtle begin to move forward. To stop it, run the same command, but with a `0` for the
linear `x` velocity instead:
```bash
rostopic pub /oswin/velocity geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Try playing around with the the `x` part of `linear` and the `z` part of `angular`, and seeing how the turtle
moves.

## ROS messages
Now that we've seen how to use the `rostopic` tool, let's look more at the ROS **messages** themselves. So far, we've
seen the `geometry_msgs/Twist` message, but how do they work?

### `.msg` files
ROS Messages are defined using `.msg` files. For example, the
[`twist.msg`](https://github.com/ros/common_msgs/blob/jade-devel/geometry_msgs/msg/Twist.msg) looks like:
```.msg
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

Each line represents one field in the message, with the type on the left and the
name on the right. In this case, the `twist.msg` message has two fields:
"linear" of type `Vector3`, and "angular" of type `Vector3`. The `Vector3` type that
this message refers to is another type that's defined
[somewhere else](https://github.com/ros/common_msgs/blob/jade-devel/geometry_msgs/msg/Vector3.msg):
```.msg
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
```

In general, messages are described by other messages within them.

## Summary
This exercise, we learned about:
- [Launching ROS Nodes](#ros-nodes-and-launching-buzzsim)
    + `rosrun <package-name> <node-name>`, ie. `rosrun igvc_buzzsim buzzsim`
- [ROS Topics and `teleop_twist_keyboard`](#ros-topics-and-keyboard-teleop)
    + A ROS topic is like an address that messages are sent to
    + A ROS node can **subscribe** and **publish** messages to any topic
    + Each topic has a message type.
- [The `rostopic` tool](#the-rostopic-tool)
    + `rostopic list` to list available topics
    + `rostopic echo` to listen to topics
    + `rostopic pub` to publish to topics
