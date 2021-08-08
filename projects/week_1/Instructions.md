# Week 1 Project: Coordinate Frame Transformations

## Background

Explain context and purpose of the exercise

## How to Run

Explain how to launch relevant files. Demonstrate that the robot can't do the desired behavior with the starter code.

## Instructions

Provide the step by step instructions for modifying the code

1. Run the project launch file, look at aruco tag debug image in rqt

1. Implement our rotation matrix helper function
   * Replace identity values with rotation matrix values in `getRotationMatrixForOpticalFrame()`.

1. Prepare loop over old tags
   * use classic for loop syntax
   * Declare new tag message & copy id
   * Push new tag message into new array message

1. Transform tag position

1. Transform tag orientation

1. Run final code and see great results!
   * Show how to run joystick code to drive robot around