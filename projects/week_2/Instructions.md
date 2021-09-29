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
  - [1.1 Color Spaces](#11-color-spaces)
- [2 Running this project](#2-running-this-project)
- [3 Instructions](#3-instructions)
  - [3.1 Get the latest starter code](#31-get-the-latest-starter-code)
  - [3.2 Create new files](#32-create-new-files)
  - [3.3 Declare functions](#33-declare-functions)
  - [3.4 Convert Image to HSV in `FindColors`](#34-convert-image-to-hsv-in-findcolors)
  - [3.5 Check Pixel Colors in `FindColors`](#35-check-pixel-colors-in-findcolors)
  - [3.6 Implement `ReprojectToGroundPlane`](#36-implement-reprojecttogroundplane)
  - [3.7 Call our functions in `ObstacleDetector`](#37-call-our-functions-in-obstacledetector)
  - [3.8 Setup publisher](#38-setup-publisher)
  - [3.9 Setup subscriber](#39-setup-subscriber)
  - [3.10 Build and test](#310-build-and-test)
  - [3.11 Simplifying our code with library functions](#311-simplifying-our-code-with-library-functions)
  - [3.12 Commit your new code in git](#312-commit-your-new-code-in-git)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## 1 Background

Color detection and homography are very common tasks for any robot that uses a camera to follow (or avoid) markings on the floor. In this project, we're going to make our robot aware of the "rocks" in our challenge mission, represented by red blobs on the challenge mat. You'll be implementing functions to isolate the red color in the image and use homography to reproject the camera's image into a top-down map for the robot.

For this project, we'll be working in the [obstacle_detector](../../obstacle_detector) package. The starter code for this project has defined a node for you. This node, `ObstacleDetector`, will subscribe to an image topic. Each image that comes across that topic will need to be filtered to highlight the pixels that match our target color. You'll then re-project the image with a homography matrix we provide to turn the image into a top-down map that the node will publish as an occupancy grid message.

### 1.1 Color Spaces

This project involves detecting colors in an image. When doing operations on the color data of our images, the color space we use is important. Different color spaces can make different operations on the colors easier or harder. The RGB image space you're probably most familiar with is great for storing images that will be displayed with a digitial screen. RGB breaks our colors into red, green, and blue components. This is convenient as most displays use red, green, and blue lights in each pixel to render the image. This is not a very helpful color space for color detection though, because it can be complicated to define the volume occupied by a single, conceptual color.

A color space that's easier to work with for color detection is "HSV". This color space still uses three channels, but now these channels represent the hue, saturation, and value properties of the color. Hue tells us the dominant frequency of the light, or the part of light that we most associate with color. Hue is a cylcic value often associated with angles. Red lies at the wrap-around point (0 or 360 degrees). Saturation tells us how "pure" the color is, with low saturation being completely grayscale, and high saturation being the pure color. Value tells us the brightness of the color. Low values are dark colors, with 0 value being black.

This color space isn't perfect. Note that there are certain kinds of singularities where multiple HSV values can be used to represent the exact same color, such as white or black. In practice, however, it's very useful for detecting most colors.

![The HSV cylinder](https://upload.wikimedia.org/wikipedia/commons/thumb/3/33/HSV_color_solid_cylinder_saturation_gray.png/320px-HSV_color_solid_cylinder_saturation_gray.png)

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

### 3.2 Create new files

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

Find the student code block in the call to `add_library`. Add our new implementation file, `src/student_functions.cpp` to this function call.

**Note:** We don't need to add our header file, because it gets compiled as part of the files that include it.

### 3.3 Declare functions

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

### 3.4 Convert Image to HSV in `FindColors`

It's now time to implement our `FindColors` function. This function will highlight every pixel whose color is in the given range of colors. In the returned image, any pixels whose input color values were within the range will be set to white, and the rest will be set to black.

The first thing `FindColors` needs to do is convert the input image to the HSV color space. This is the color space used to define our target color range. We'll start by opening up [student_functions.cpp](../../obstacle_detector/src/student_functions.cpp) and adding an empty definition for `FindColors`:

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

### 3.5 Check Pixel Colors in `FindColors`

The second step in `FindColors` is populating our single-channel output image based on the color values in the HSV image. We'll iterate over the HSV image, checking each pixel. For every pixel in our target color range, the corresponding pixel in the output image will be set to white (255). All other pixels in the output image will be set to black (0).

After our call to `cv::cvtColor`, declare another `cv::Mat` variable that will hold our output image, named `output`. We'll initialize this variable such that its size matches our input image's size. `CV_8UC1` tells OpenCV that this image will have one channel of unsigned, 8-bit values. (Our color images so far have been using `CV_8UC3` to store three channels of unsigned, 8-bit values).

```c++
cv::Mat output(input.size(), CV_8UC1);
```

Now we need to iterate over each pixel position in our images. Create two nested loops. The first loop should iterate over each row index, from 0 to `input_hsv.rows`. The second loop should iterate over each column index, from 0 to `input_hsv.cols`. You can name the row index variable `r` and the column index variable `c`.

In the body of our column loop, we can get the color at that position using the `at` function of `cv::Mat`:

```c++
const auto input_color = input_hsv.at<cv::Vec3b>(r,c);
```

Note that we're asking for the value at that position as a `cv::Vec3b` so it's easy to compare against our range variables. Each `cv::Vec3b` holds the three channel values that represent the color value at a given pixel as a vector (in the math sense, not `std::vector`) of bytes. To compare scalars, we can compare each of the channel values individually. `cv::Vec3b` provides a square bracket operator so we can access each channel value by its index. For example, to check if channel 0 of our input color is in our target range, we could write this:

```c++
input_color[0] >= range_min[0] && input_color[0] <= range_max[0]
```

Create an if statement that checks if all three channels of `input_color` are within our target range. Inside the body of this if statment, set the current position of our output image to 255:

```c++
output.at<uint8_t>(r,c) = 255;
```

Add an else block to our if statement. In this branch, set the output value to 0.

Finally, after the end of the nested loops, return our `output` image.

### 3.6 Implement `ReprojectToGroundPlane`

`ReprojectToGroundPlane` warps `input` using the homography defined by the `homography` matrix. This function will create an output image with the size given by `map_size`. It will then iterate over each pixel position in the output image and calculate the corresponding pixel position in the input image with the homography matrix. Finally, it copies the value from the source position in the source image to the destination position in the output image. Any pixels that get mapped outside of the bounds of the source image will be set to 127 in the output image.

Let's start by declaring our output image variable. We'll call it `output` and use `cv::Mat` as its type. Initialize it by passing `map_size` and `CV_8UC1` to its constructor. `map_size` just holds the desired height and width for our output image.

Next, setup two nested loops to iterate over all pixel positions in `output`. The first loop should iterate from `y = 0` to `y = output.rows-1`. The second loop should iterator from `x = 0` to `x = output.cols-1`.

In the body of our inner nested loop, we're going do these steps:

1. Create a homogeneous point for our current output position
1. Calculate the homogeneous point for the source image with the homography
1. Create `cv::Point2i` points for the source and destination positions
1. If source point is within bounds of source image, copy the color value to output image
1. If source point is outside bounds of source image, set output position to 127

Start by declaring a constant `cv::Vec3d` named `dest_vec`. Initialize it by passing `x`, `y`, and `1` to the constructor.

Then, declare a constant `cv::Vec3d` named `src_vec`. Initialize this with the result of multiplying the inverse of `homography` by `dest_vec`. Unfortunately, to store this result as a `cv::Vec3d`, we first need to convert it to a `cv::Mat1d`. This is just a quirk of the way OpenCV's types work.

```c++
const cv::Vec3d src_vec = cv::Mat1d(homography.inv() * dest_vec);
```

Next, create two constant `cv::Point2i` variables named `dest_point` and `src_point`. `dest_point` should just be initialized with `x` and `y`. `src_point` should be such that its x value is `src_vec[0] / src_vec[2]` and its y value is `src_vec[1] / src_vec[2]`. This division re-normalizes the homogeneous coordinate in `src_vec` so we can get back to 2D coordinates.

Now we have the source and destination points. We just need to check if our source point is actually within the bounds of our source image. `cv::Point2i` gives us the `inside` function that can tell us if the given point is within a `cv::Rect` rectangle. We can create a `cv::Rect` from our input image with `cv::Rect(cv::Point(), input.size())`. Set up an if statement that checks the result of calling `inside` on `src_point` with the `cv::Rect` just described.

Inside of the if statement, copy the value from `input` to `output`:

```c++
output.at<uint8_t>(dest_point) = input.at<uint8_t>(src_point);
```

Add an else branch that sets the output location to 127:

```c++
output.at<uint8_t>(dest_point) = 127;
```

Finally, after the end of both nested loops, return `output`.

### 3.7 Call our functions in `ObstacleDetector`

Both of our key functions are now implemented, so it's time to use them by calling them within `ObstacleDetector`. In [obstacle_detector.cpp](../../obstacle_detector/src/obstacle_detector.cpp), find the student code comment block at the end of the existing set of `#include` lines. Add an `#include` line for your header, "student_functions.hpp".

Next, find the student code comments that include `// Call FindColors()`. Within that comment block, you'll see a declared but uninitialized variable named `detected_colors`. Add an initializer for this variable that calls `FindColors`. The input image for `FindColors` is `cv_image->image`, and the color range is given by `min_color` and `max_color`.

In the same file, find the student code comment block that includes `// Call ReprojectToGroundPlane`. Again, you'll see an uninitialized variable, this time named `projected_colors`. Initialize it by calling `ReprojectToGroundPlane`. The input image here is `detected_colors`. The homography matrix is called `homography`, and the map size is called `map_size`.

### 3.8 Setup publisher

The core logic of our obstacle detection node is ready, but it doesn't actually connect to any ROS topics. We'll start by setting up the publisher that will publish `nav_msgs::msg::OccupancyGrid` messages to the `"~/occupancy_grid"` topic.

Find the student code comment block that includes `// Declare subscriber and publisher members`. With this block, declare a new member variable of type `rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr`, named `occupancy_grid_publisher_`.

Up in the `ObstacleDetector` constructor, find the student code comment block that includes `// Initialize publisher and subscriber`. Set `occupancy_grid_publisher_` to the returned value of `create_publisher`. The topic should be `"~/occupancy_grid"` and the quality of service setting should be `rclcpp::SystemDefaultsQoS()`. Don't forget to specify the message type (`nav_msgs::msg::OccupancyGrid`) as a template parameter.

Now scroll down to find the student code comment block that includes `// Publish occupancy_grid_msg`. Here, call `publish` on `occupancy_grid_publisher_`, passing it `occupancy_grid_msg`. Remember, our publisher object is a shared pointer, so we'll use the arrow syntax (`->`) for accessing the member function.

### 3.9 Setup subscriber

Now that our publisher is setup to get the map data out of our node, we need to setup the subscriber that will pull data into the node. There is a library called "image_transport" that gives us special publisher and subscriber types for efficiently working with image messages and cameras data. We'll be using image_transport's `CameraSubscriber`. This object subscribes to both the image topic and the camera info topic that includes metadata like our cameras intrinsics matrix (sometimes called the "K matrix"). We can then get both the image and camera info data in the same subscriber.

Back in the `// Declare subscriber and publisher members` comment block, declare another member variable of type `image_transport::CameraSubscriber` named `camera_subscriber_`. 

In the `// Initialize publisher and subscriber` comment block, use `image_transport::create_camera_subscription` to initialize `camera_subscriber_`. This is a slightly different interface than the standard `create_subscription` method.

```c++
camera_subscriber_ = image_transport::create_camera_subscription(
      this, "/camera/image_raw",
      std::bind(
        &ObstacleDetector::ImageCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
```

The first argument, `this` gives the function a reference to our node object.

`"/camera/image_raw"` is the image topic we want to subscribe to. The camera info topic, `"/camera/camera_info"`, will be derived from the given image topic.

The call to `std::bind` defines our subscription callback.

`"raw"` is called the "transport hint" and tells image_transport that we're interested in the raw, uncompressed version of the images.

Finally, `rclcpp::SensorDataQoS()` sets our quality of service settings to good defaults for sensor data.

That's it! Our subscription callback, `ImageCallback` already exists, so we don't need any more code to make our subscriber work. All of our ROS inputs and outputs are ready.

### 3.10 Build and test

Build your training workspace with `colcon build`, and run the project using the instructions in [section 2](#2-running-this-project). Debug any problems you find before continuing. Once you do have your output working and looking like the examples in section 2, congratulations! You've just implemented two perception functions that have been the backbone of many robots! Both RoboNav and RoboRacing have won trophies using code that largely boiled down to these two functions. They are unreasonably good at what they do for how simple they are.

### 3.11 Simplifying our code with library functions

Of course, because these two operations are so common, there are functions that come with OpenCV to do exactly these steps. We can rewrite both of our nested loop pairs with single calls to some library functions.

In your `FindColors` implementation, the nested loops can be replaced with `cv::inRange`. This does exactly the same thing our loops did.

```c++
cv::inRange(input_hsv, range_min, range_max, output);
```

In `ReprojectToGroundPlane`, the nested loops can be replaced by calling `warpPerspective`. Again, this function does exactly the same thing that our loops implemented.

```c++
cv::warpPerspective(
    input, output, homography, map_size, cv::INTER_NEAREST, cv::BORDER_CONSTANT,
    cv::Scalar(127));
```

While implementing these algorithms ourselves is a good way to understand what they do, in "real code" there's almost never a reason to do so. Using library functions like these makes our code much easier to write and maintain. If you've done everything completely right, you should be able to switch between your loops and the library functions without noticing a difference in the node's behavior.

### 3.12 Commit your new code in git

Once you've got your code for this project working, use the command below to commit it into git. This will make it easier to grab changes to the starter code for the remaining projects.

```bash
$ git commit -a -m "My project 2 code."
```
