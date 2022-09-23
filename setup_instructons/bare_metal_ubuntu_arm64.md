# Bare Metal Ubuntu Setup (arm64)

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [1 Install ROS](#1-install-ros)
- [2 Install STSL](#2-install-stsl)
- [3 Clone software_training Repository](#3-clone-software_training-repository)
- [4 Install Gazebo](#4-install-gazebo)
- [5 Clone ROS Gazebo packages](#5-clone-ros-gazebo-packages)
- [6 Install ROS dependencies](#6-install-ros-dependencies)
- [7 Build training workspace](#7-build-training-workspace)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## 1 Install ROS

1. Add ROS package repository

   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release

   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

1. Update APT cache and upgrade system packages

   ```bash
   sudo apt update
   sudo apt upgrade
   ```

1. Install core ROS packages, rosdep tool, and colcon build tool

   ```bash
   sudo apt install ros-humble-desktop python3-rosdep2 python3-colcon-common-extensions
   ```

## 2 Install STSL

1. Get the public key

   ```bash
   wget -qO - https://stslaptstorage.z13.web.core.windows.net/pubkey.txt | sudo apt-key add -
   ```

1. Add the STSL package repository

   ```bash
   sudo apt-add-repository "deb https://stslaptstorage.z13.web.core.windows.net/ jammy main"
   ```

1. Install the STSL ackages

   ```bash
   sudo apt install ros-humble-stsl-desktop
   ```

## 3 Clone software_training Repository

1. Create training workspace directory

   ```bash
   cd ~
   mkdir -p training_ws/src
   ```

1. Clone the repo

   ```bash
   cd ~/training_ws/src
   git clone https://github.com/RoboJackets/software-training.git
   ````

## 4 Install Gazebo

1. Install the arm64 version of gazebo

   ```bash
   sudo add-apt-repository ppa:openrobotics/gazebo11-non-amd64

   sudo apt update

   sudo apt install gazebo
   ```

## 5 Clone ROS Gazebo packages

1. Clone ros_gazebo_pkgs repository

   ```bash
   cd ~/training_ws/src
   git clone --branch=3.7.0 https://github.com/ros-simulation/gazebo_ros_pkgs.git
   ```

## 6 Install ROS dependencies

1. Initialize rosdep

   ```bash
   sudo rosdep init
   rosdep update
   ```

1. Run rosdep

   ```bash
   rosdep install --from-paths . --ignore-src -y
   ```

## 7 Build training workspace

1. Go to training workspace directory

   ```bash
   cd ~/training_ws
   ```

1. Source ROS system underlay script

   ```bash
   source /opt/ros/humble/setup.bash
   ```

1. Run colcon

   ```bash
   colcon build
   ```

   If everything worked, you should see the last line of output look like this (the build time may be different for you):

   ```bash
   Summary: 9 packages finished [1min 46s]
   ```