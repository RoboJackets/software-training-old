# Exercise 6.1
Welcome to Week 6 of ROS training exercises! We'll be learning about **bag files**.

## Using Bag Files
#### What Are Bag Files?
A ROS _bag file_ is used to store multiple topic's message data. Bags are the primary mechanism
in ROS for data logging and are typically created by [rosbag](http://wiki.ros.org/rosbag),
which subscribes to one or more ROS topics, and _stores_ the serialized message data in a file as it is
received. These bag files can also be _played back_ in ROS to the same topics they were recorded from,
or even remapped to new topics.

<img width="860" height="300" src="https://i.imgur.com/HxoUDXd.png"/>

#### Why Do We Use Bag Files?
We often use these bag files to play back the recorded topic data for testing the behavior on new features.
So a common use case for bag files is while we outside testing some feature on the robot, we often
want to record and store the data on a number of topics so that later we can play back this recorded
data for testing and debugging. _Warning:_ Bag files are usually pretty big... Usually
about 1-3 GB for a single bag file so make sure you have enough room on your system
before you download / uncompress one.

#### Running Bag Files
In order to play back a bag file simply do:
```rosbag play <your bagfile>```
In order to make the bag file run repeatly simple add the loop flag `-l`:
```rosbag play <your bagfile> -l```
You can also press `[Space]` in the terminal running the bag file in order to pause/play the recording.
Remember to run `roscore` before playing the bag file!

## Image Visualization
Just like how we used `rviz` for visualization of the world in previous exercises, now we
are going to use `rqt_image_view` for displaying images. This just a simple GUI for looking
at what image is being published on any topic. The command for running it is:
```rqt_image_view```

<a href="https://imgur.com/TH2EAS3"><img src="https://i.imgur.com/TH2EAS3.png" title="source: imgur.com" /></a>

 Practice working with bag files by running the
 [start_light bag file](../bag/start_light.bag) by playing it on loop while using `rqt_image_view` for visualization.

 ## Excerise
 Part 1: <br>
 1. Run bag file `start_light.bag` file on loop maunally (ex. `rosbag ...`)
    - <details> <summary>Hint</summary> rosbag play bag/start_light.bag -l
    </details>
 2. Visualize the results by looking at `rviz_image_view`


 Part 2: <br>
 1. Now record your own bag file (~5 sec) of you controlling the turtle in `buzzsim` with the `teleop_twist_keyboard` node
    - Only record `/oswin/velocity` topic
    - <details>
        <summary>Hint</summary> Four Terminals: <br>
        -roscore <br>
        -rosrun buzzsim buzzsim <br>
        -rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity <br>
        -rosbag record -O buzzsim_example.bag /oswin/velocity
    </details>

 2. Play back your bag file with twist messages on loop and check that buzzsim is excuting the recorded actions
    - <details> <summary>Hint</summary> rosbag play buzzsim_example.bag -l
     </details>
