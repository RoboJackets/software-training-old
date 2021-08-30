<!--
STOP
We strongly recommend viewing this file with a rendered markdown viewer. You can do this by:
 - Opening this file in the GitHub web viewer
 - Pressing Ctrl+Shift+V in Visual Studio Code
 - Opening this file in any other markdown viewer you prefer
-->

# Week 2 Project: Coordinate Frame Transformations

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Contents

- [1 Background](#1-background)
- [2 Running this project](#2-running-this-project)
- [3 Instructions](#3-instructions)
  - [3.1 Create new files](#31-create-new-files)
  - [3.2 Declare functions](#32-declare-functions)
  - [3.3 Implement `FindColors`](#33-implement-findcolors)
  - [3.4 Implement `ReprojectToGroundPlane`](#34-implement-reprojecttogroundplane)
  - [3.5 Call `FindColors` in `ObstacleDetector`](#35-call-findcolors-in-obstacledetector)
  - [3.6 Call `ReprojectToGroundPlane` in `ObstacleDetector`](#36-call-reprojecttogroundplane-in-obstacledetector)
  - [3.7 Setup publisher](#37-setup-publisher)
  - [3.8 Setup subscriber](#38-setup-subscriber)
  - [3.9 Build and test](#39-build-and-test)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## 1 Background

Color detection and homography are very common tasks for any robot that uses a camera to follow (or avoid) markings on the floor. In this project, we're going to make our robot aware of the "rocks" in our challenge mission, represented by red blobs on the challenge mat. You'll be implementing functions to isolate the red color in the image and use homography to reproject the camera's image into a top-down map for the robot.

For this project, we'll be working in the [obstacle_detector](../../obstacle_detector) package. The starter code for this project has defined a node for you. This node, `ObstacleDetector`, will subscribe to an image topic. Each image that comes across that topic will need to be filtered to highlight the pixels that match our target color. You'll then re-project the image with a homography matrix we provide to turn the image into a top-down map that the node will publish as an occupancy grid message.

## 2 Running this project

You can run this week's project using the week 2 launch file. This will startup the simulator, rviz, and your obstacle detection code.

```bash
$ ros2 launch rj_training_bringup week_2.launch.xml
```

To move the robot around and test your code at different positions, use the same joystick or keyboard control launch files as you used in week 1.

If you run it now, you'll see rviz complain that no messages are being published on our occupancy grid topic.

![Screenshot of rviz showing warning message](RvizNoGrid.png)

Once you've got your code working, you should see a map show up in rviz's 3D viewport. The black areas are marked obstacles and should match the shape of the red rocks on the challenge mat. The white areas are free of obstacles. The grey area is the part of the reprojected image that we had no original pixel data for, so the robot doesn't know what's in that area.

![Screenshot of rviz showing working obstacle detection](WorkingDetection.png)

## 3 Instructions

### 3.1 Create new files

You'll be implementing the two steps of this project, color detection and re-projection, as new fuctions defined in separate files from the node's main file. The first thing we need to do is add these files to our package.

Go to the ["src" folder](../../obstacle_detector/src), in the obstacle_detector package. Add two new files: "student_functions.cpp" and "student_functions.hpp".

In the implementation file (student_functions.cpp), we need to include our new header file. Add an include statement at the top of the file to include the header file we just created.

<details>
<summary><b>Hint:</b> How to include a header</summary>
<p>To include a header file named "my_header.hpp", we'd write the code below. Note that the use of quotes ("") instead of angle brackets (<>) indicates that this header is part of our current package.</p>
<code>#include "my_header.hpp"</code>
</details>

In our new header file (student_functions.hpp), we need to add header guards. Add the three lines that define the header guards for this file. Remember that header guards look like this:

```c++
#ifndef STUDENT_FUNCTIONS_HPP  // Check that the unique macro has not already been defined
#define STUDENT_FUNCTIONS_HPP  // Define the unique macro

#endif  // Close the pre-processor if block
```

Finally, to include our new files into our package's build rules, we need to edit obstacle_detector's [CMakeLists.txt](../../obstacle_detector/CMakeLists.txt).

Find the student code block in the call to `add_library`. Add our new implementation file, `src/student_code.cpp` to this function call.

**Note:** We don't need to add our header file, because it gets compiled as part of the files that include it. Most C++ projects omit header files from the build rules unless there are special technical reasons that require them to be there.

### 3.2 Declare functions

Let's now declare the two functions we'll be creating for this project. First, because our functions will be using objects from the [OpenCV library](https://opencv.org/) for computer vision, we need to include a header from that library.

Add an include directive for "opencv2/opencv.hpp". Remember that all code in our header file should go inside of the header guard if block (between `#define` and `#endif`).

**Tip:** Because the OpenCV library is not part of our package, this include directive should use the angle bracket style.

Next, add the declaration for our first function. This function should be named `FindColors`. Its return type should be `cv::Mat`, and it should take the following parameters:

1. A constant `cv::Mat` named `input`
1. A constant `cv::Scalar` named `range_min`
1. A constant `cv::Scalar` named `range_max`

Finally, add the declaration for our second function. This function should be named `ReprojectToGroundPlane`. Its return type should be `cv::Mat`, and it should take these parameters:

1. A constant `cv::Mat` named `input`
1. A constant `cv::Mat` named `homography`
1. A constant `cv::Size` named `map_size`

### 3.3 Implement `FindColors`

It's now time to implement our `FindColors` function. This function needs to check each pixel in `input` to see if the color value at that pixel is within the range defined by `range_min` and `range_max`. In our output image, any pixels whose color values were within the range will be set to white, and the rest will be set to black.

When doing operations on the color data of our images, the color space we use is important. Different color spaces can make different operations on the colors easier or harder. The RGB image space you're probably most familiar with is great for storing images that will be displayed with a digitial screen. RGB breaks our colors into red, green, and blue components. This is convenient as most displays use red, green, and blue lights in each pixel to render the image. This is not a very helpful color space for color detection though, because it can be complicated to define the volume occupied by a single, conceptual color.

A color space that's easier to work with for color detection is "HSV". This color space still uses three channels, but now these channels represent the hue, saturation, and value properties of the color. Hue tells us the dominant frequency of the light, or the part of light that we most associate with color. Hue is a cylcic value often associated with angles. Red lies at the wrap-around point (0 or 360 degrees). Saturation tells us how "pure" the color is, with low saturation being completely grayscale, and high saturation being the pure color. Value tells us the brightness of the color. Low values are dark colors, with 0 value being black.

This color space isn't perfect. Note that there are certain kinds of singularities where multiple HSV values can be used to represent the exact same color, such as white or black. In practice, however, it's very useful for detecting most colors.

![The HSV cylinder](https://upload.wikimedia.org/wikipedia/commons/thumb/3/33/HSV_color_solid_cylinder_saturation_gray.png/320px-HSV_color_solid_cylinder_saturation_gray.png)

Thus, our implementation of `FindColors` will use two steps: convert our input image to the HSV color space then populate a single-channel output image based on the HSV value at each pixel. We'll start by opening up [student_functions.cpp](../../obstacle_detector/src/student_functions.cpp) and adding an empty definition for `FindColors`:

```c++
cv::Mat FindColors(const cv::Mat input, const cv::Scalar range_min, const cv::Scalar range_max)
{
}
```

Now declare a variable to hold the HSV version of our input image. This should be a `cv::Mat` variable called `input_hsv`. Then, call the `cvtColor` function from OpenCV to do the conversion to HSV:

```c++
cv::Mat input_hsv;
cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);
```

Note that the conversion type uses "BGR". This color space uses the same channels as RGB, just in the opposite order in the image data. OpenCV uses BGR as the default color space for color images.

Next, declare another `cv::Mat` variable that will hold our output image, named `outupt`. We'll initialise this variable to be the same size as our input image while holding one channel of unsigned, 8-bit values.

```c++
cv::Mat output(input.size(), CV_8UC1);
```

Now we need to iterate over each pixel position in our images. Create two nested loops. The first loop should iterate over each row index, from 0 to `input_hsv.rows`. The second loop should iterate over each column index, from 0 to `input_hsv.cols`. You can name the row index variable `r` and the column index variable `c`.

In the body of our column loop, we can get the color at that position using the `at` function of `cv::Mat`:

```c++
const auto input_color = input_hsv.at<cv::Scalar>(r,c);
```

Note that we're asking for the value at that position as a `cv::Scalar` so it's easy to compare against our range variables. Each `cv::Scalar` holds the three channel values that represent the color value at a given pixel. To compare scalars, we can compare each of the channel values individually. `cv::Scalar` provides a square bracket operator so we can access each channel value by its index. For example, to check if channel 0 of our input color is in our target range, we could write this:

```c++
input_color[0] >= range_min[0] && input_color[0] <= range_max[0]
```

Create an if statement that checks if all three channels of `input_color` are within our target range. Inside the body of this if statment, set the current position of our output image to 255:

```c++
output.at<uint8_t>(r,c) = 255;
```

Add an else block to our if statement. In this branch, set the output value to 0.

Finally, after the end of the nested loops, return our `output` image.

### 3.4 Implement `ReprojectToGroundPlane`

### 3.5 Call `FindColors` in `ObstacleDetector`

### 3.6 Call `ReprojectToGroundPlane` in `ObstacleDetector`

### 3.7 Setup publisher

### 3.8 Setup subscriber

### 3.9 Build and test
