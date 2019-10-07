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
1. `./runnable.out <move time> <move power> <min circle size> <min circularity> <min brightness value>`
    - `./runnable.out 0 0 0 0 0` should execute but won't do anything useful until we add more code and tune these arguments

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


