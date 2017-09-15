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

## How to Rebuild the STSL ##

Binaries and headers for the STSL are provided in the STSL folder. These should work for most circumstances. If you experience problems building the demos with the provided binaries, you may need to rebuild the STSL from source.

1. Clone the STSL source from [GitHub](https://github.com/RoboJackets/stsl).

   ```git clone https://github.com/RoboJackets/stsl```

2. Navigate to the STSL source directory, create a _build_ directory, and navigate into it

   ```cd stsl && mkdir build && cd build```
   
3. Configure the project with CMake

   ```cmake ..```
   
4. Build the STSL

   ```make```
   
5. Copy the binaries back to the hardware_applications project.
   
   __Windows__
   
   Copy _libSTSL.a_ from _build_ to _hardware_applications/STSL/lib/WINDOWS_.
   
   __Linux__
   
   Copy _libSTSL.a_ from _build_ to _hardware_applications/STSL/lib/LINUX_.

   __OS X__
   
   Copy _libSTSL.a_ from _build_ to _hardware_applications/STSL/lib/OSX_.