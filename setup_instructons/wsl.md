# Windows Subsystem for Linux

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [1 (optional) Install Windows Terminal](#1-optional-install-windows-terminal)
- [2 Install WSL](#2-install-wsl)
- [3 Install ROS](#3-install-ros)
- [4 Install STSL](#4-install-stsl)
- [5 Clone software_training Repository](#5-clone-software_training-repository)
- [6 Install ROS dependencies](#6-install-ros-dependencies)
- [7 Build training workspace](#7-build-training-workspace)
- [8 Install VS Code](#8-install-vs-code)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->


## 1 (optional) Install Windows Terminal

While not required, the Windows Terminal app offers a much nicer experience than the default Command Prompt or PowerShell terminal apps.

You can install the Windows Terminal app from the Microsoft Store: [App Store Page](https://www.microsoft.com/store/productId/9N0DX20HK701)

## 2 Install WSL

1. Open a PowerShell instance.

   Either search the start menu for PowerShell or open the Windows Terminal app and select "PowerShell" profile from the drop down at the top.

1. Run the install command

   ```
   wsl --install --distribution Ubuntu-22.04
   ```

## 3 Install ROS

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

## 4 Install STSL

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

## 5 Clone software_training Repository

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

## 8 Install VS Code

1. Install VS Code from the Microsoft Store: [App Store Page](https://apps.microsoft.com/store/detail/XP9KHM4BK9FZ7Q)

1. Launch VS Code

1. Install the "Remote - WSL" extension by clicking "Install" on [this page](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)

1. Click the green, "><" button in the bottom left corner. Select "New WSL Window". This will start a new instance of VS Code that edits and runs its code on your WSL VM. It may take several minutes to install the needed server and support software in the VM.

1. Install the "C/C++ Extension Pack" by clicking "Install" on [this page](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
