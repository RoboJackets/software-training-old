# Exercise 4.2

# Exercise Objective
You will be subscribing to some topics using the rqt tools you learned about in the videos

## Running the node
We have coded up a novel node that will publish some topics, your goal is to
figure out what those topics are and use the rqt tools to plot what you are seeing.

To run our node type

```
rosrun week_4_exercises rqt_plot_node
```

don't forget to have roscore running in the background

# rqt_plot
There are 3 topics that you should be able to plot

```
/cosine
/sine
/odom
```

Try running the rqt_plot utility while also running out node

```
rosrun rqt_plot rqt_plot
```

You should be easily able to plot what you are seeing on /cosine and /sine.
Verify that you see plots that are cosine and sine through time.

## Plotting complicated messages
Now /odom should not work directly, we will have to inspect the message to find out why.
I have included a printout of what happens when I echo that message.

```
header:
  seq: 1700
  stamp:
    secs: 1600651228
    nsecs: 437046052
  frame_id: ''
child_frame_id: ''
pose:
  pose:
    position:
      x: 0.471422085606
      y: 0.881907714675
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.471422085606
      y: 0.881907714675
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

This is a single message, rqt does not know how to plot this entire thing, but we
can start to peel back the layers of the message to get to something it can plot.
(Yes, layers Donkey).

Something you might not know is that there is documentation for the messages, we
have a nav_msgs::Odometry message which you can find the docs [here](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html).
It is really just a message made up message, made up of messages, etc. We can
look at one message down by just adding more text after the slash like so

```
rostopic echo /odom/pose
```

that gives us

```
---
pose:
  position:
    x: 0.806085940717
    y: -0.591798492882
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

You can see that we can drill down to something we want to plot, like position.x and
position.y, something like

```
rostopic echo /odom/pose/pose/position
```

Try using that as your way to add the message to rqt_plot, it should work
and add all three x,y,z to your plot. z might not be visible at first since it is zero.
Pan the plot up a bit to see that line. You should see x and y look identical to
/cosine and /sine

# rqt_image_view
Now we will be using rqt_image_view to show an image that is being published by the node.
Leaving our node and roscore running, start the viewer by doing

```
rosrun rqt_image_view rqt_image_view
```

Now you the topic selector to grab the topic ```/robo_buzz```. You should see
our lovely mascot rotating in the image view.

That's all for this exercise.
