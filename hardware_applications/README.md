# Software Training Hardware Applications #

These files contain source code for the hardware applications demonstrated in the RoboJackets Software Training program.

These demos utilize the [Software Training Support Library (STSL)](https://github.com/RoboJackets/stsl) developped by RoboJackets.

## How to Build the Demos ##

In our training sessions, we'll be using [JetBrains' CLion IDE](https://www.jetbrains.com/clion/). Below you can find instructions on how to build the hardware application demos both with and without this IDE.

### Using CLion ###

1. Open CLion

2. If you have another project open, select File->Open...

   If you see the CLion welcome dialog, select Open Project
   
3. Once the project loads, select Run->Build

At this point, you can select a specific demo to run using the drop down and other controls in the top right of the IDE window.

## How to Run a Demo ##

1. In a terminal, navigate to the hardware_applications directory.

2. Create a _build_ directory and navigate into it.

   ```mkdir build && cd build```
   
3. Configure the project with CMake

   ```cmake ..```
   
4. Build all demos

   ```make```
   
## Troubleshooting ##

### Linux ###

__Error__: Could not find UDev library

__Solution__: Install libudev. ie: `sudo apt-get install libudev-dev`

## How to Rebuild the STSL ##

As of Week 2, 2017, the STSL is now automatically downloaded and built in your cmake build directory. If you encounter a problem you believe can be fixed by rebuilding the STSL, simply clean and rebuild the hardware-applications project.