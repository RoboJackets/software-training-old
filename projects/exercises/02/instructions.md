# Exercise 02

## Concept

We will be writing a program that uses the robot's camera to point towards a circular target. This target is expected to be a dark circle on a light background or vice versa. Each time the robot gets a new frame from the camera, it will:

1. identify the boundaries between the dark and light parts of the image
1. determine whether each boundary is a circle or not
1. find the center of the circle, and see if it's in the left or right half of the image
1. turn the robot a small amount toward the detected circle

Programming this is not a trivial task -- it took the instructors several hours to create the first working solution -- so this will be as much of a walkthrough as it is a coding challenge. This guide will help you get it working.

## Running this code

1. https://find-robot.robojackets.org
1. As always, make sure you run the code using the "C++ (RoboJackets)" runner
1. The code has several command-line arguments that need to be provided. When the runner finishes compiling the code, it will run it without the arguments, and the program will return right away with exit code 1.
1. Open a terminal in the IDE and `cd ~/software-training/projects/exercises/02`
1. `./runnable.out <move time> <move power> <min circle size> <min circularity> <max brightness value>`
    - `./runnable.out 0 0 0 0 0` should execute but won't do anything useful until we add more code and tune these arguments
1. Compiling code on the robots takes a long time, partially because there are other processes running. Run `sudo service avahi-daemon stop` to kill a CPU-intensive background process that starts automatically with the robots. We are in the process of getting this to happen automatically. The instructors can give you the robot password for this command.

## 1. View an Image

At the end of the while loop, uncomment the `cv::imwrite` line and the `robot.wait` line after. This will save the image `img` to the robot's storage, and you can view it in the IDE. The wait makes it less likely that you stop the program (with `Ctrl+C`) while it is in the process of saving the image.

## 2. Blurring

Often, the first step in a computer vision algorithm is to blur the image. This will smooth out small inconsistencies, "noise" in the camera sensor, etc.

We recommend using `cv::GaussianBlur` (https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1) to accomplish this. To filter with a strength of 1, call `cv::GaussianBlur(img, img, cv::Size(3,3), 1, 1);`. In general, if `ksize` is k by k, then a good sigma value is (k - 1) / 2.

Run the program and inspect img.jpg to see the blurring effect.

## 3. Conversion to HSV

This part shows off OpenCV's ability to reprsent images in different color spaces. A color space is a way of encoding a color with numbers. BGR is the default color space for OpenCV, where each color is a blue, green, and red component. Hue, saturation, and value (HSV) is another common color space. We are interested in the "value" channel, which is large for bright pixels and small for dark ones.

Use `cv::cvtColor` (https://docs.opencv.org/master/d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab). The parameter "code" specifies what conversion to do. We want `cv::COLOR_BGR2HSV` as the code. Don't worry about dstCn, leave it at 0.

If you view img.jpg when an HSV image was saved, the colors will look weird. This is because the image viewer will load it as BGR or RGB colors.

## 4. Thresholding

Now we will "threshold" the image, turning it from a 3-channel HSV image (where each pixel has 3 values 0-255) into a 1-channel binary image (where each pixel has 1 value, true or false). The function `cv::inRange` (https://docs.opencv.org/master/d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981) will take an image and make a binary image that is true where the input is between an lower and upper bound. For `lowerb` and `upperb`, use a `cv::Scalar(h, s, v)` where h, x, and v are integers. If we want all possible hues, all possible saturations, and all values below `max_value` to pass the threshold, what `Scalar`s should we use?

Be sure to utilize the `max_value` variable that is stored from command-line arguments.

View img.jpg to see the results of thresholding. If you hold up one of the circle targets in front of the camera, you should see the black circle show up white and the white background show up black. You will have to tune max_value (the last command-line argument) to get this to work. For example, you can find a value that's too big (too much white or true) and one that's too small (too much black or false) to find the circle, then take the average.

## 5. Find Contours/Outlines

Now we want to start reasoning about the shapes of the observed binary blobs. OpenCV does this using a datatype called a contour, which is just a list of points that represents the edge of a shape.

Call `cv::findContours` (https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#gae4156f04053c44f886e387cff0ef6e08) passing in `cv::RETR_LIST` for `mode` and `cv::CHAIN_APPROX_SIMPLE` for `method`. Feel free to read up on what these do if you want. The contours found by `cv::contours` should end up stored in the variable `contours` that is already in the starter code.

Have your program print `contours.size()` to the terminal to verify that it is finding edges.

## 6. find_circle_column

Now we want to filter out contours/outlines/shapes which are not circles or that are too small, and then return the x coordinate or column of the center of that circle.

### 6.1. Area

Use `cv::contourArea` (https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1) to find the area. Our contour is not oriented.

### 6.2. Perimeter

Use `cv::arcLength` (https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#ga8d26483c636be6b35c3ec6335798a47c) to find the length of the contour/outline. This is the perimeter of our closed poly-line.

### 6.3. Circularity

"Circularity" is a measure of the circle-ness of an arbitrary shape.

circularity = ![circularity definition](https://www.learnopencv.com/wp-content/uploads/2015/02/circularity.png)

Take some time to convince yourself that circles have a circularity of 1.0, and all other shapes are less than that. Then calculate circularity in your code.

### 6.4. X / Column Center

OpenCV allows you to calculate lots of information about the "moments", or measures, of shapes using the `cv::moments` function (https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139).

The returned data structure (of type `cv::Moments`) contains the zero-th through second moments of our shape. A quick google of how to use this information will tell you that the mean in the x direction is given by `moments.m10 / moments.m00`. m10 is the first moment in the x direction, and m00 is the zero-th moment or total "mass". Funny how CS borrows from physics here.

Store this value in x_center to complete `find_circle_column`.

Back in the main method, if `circle_column` is nonnegative, have your program print its value. You will have to tune the minimum circle area (3rd argument, integer) and minimum circularity (4th argument, float) to get reasonable results.

## 7. Turn the robot to face the target

We can easily determine whether the circle is in the left or the right side of the image by comparing it to the x-index of the middle column. (Hint: we know the size of the image is `img.cols`.)

The function `scoot(robot, direction)` will turn the robot counter-clockwise (toward the left side of the image) if `direction` is negative, or clockwise if positive. So we can subtract the middle column from the circle column and use this as the `direction` to turn toward the circle.

At first, `scoot` will do nothing because the move time and move power are both zero. Set these arguments on the command line, where move time is the first argument (integer number of milliseconds, start with 50) and move power is the second argument (float -1.0 to 1.0, try 0.3).

Comment out the imwrite and wait commands that we un-commented earlier. These will slow down your circle tracking code.

Tune all your parameters to try and get the best tracking performance that you can.
