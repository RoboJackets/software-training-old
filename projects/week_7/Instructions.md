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

In this project we will be writing an implementation of LQR.
Our robot has a nominal path that we will need to follow through the world.

In the final challenge for this semester, our robot will need to use every other project to complete the entire course.
The key part of that for this week is following a trajectory.


## 2 Running this project

To run this week's project, you'll need to run one launch file:

```bash
$ ros2 launch rj_training_bringup week_7.launch.xml
```

As usual, this launch file will startup the simulator, rviz, and all of the other nodes the project requires.

By default, this will use the particle filter localizer node you worked on in project 3. You can optionally use the fake localizer with the `use_fake_localizer` argument.

```bash
$ ros2 launch rj_training_bringup week_7.launch.xml use_fake_localizer:=true
```

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

### 3.10 Commit your new code in git

Once you've got your code for this project working, use the command below to commit it into git. This will make it easier to grab changes to the starter code for the remaining projects.

```bash
$ git commit -a -m "My project 6 code."
```
