# RoboJackets Software Training - Fall 2021

The RoboJackets software training program is designed to prepare you to work on software for the RoboJackets competitive teams. This program offers practical experience writing code using C++ and ROS. It also covers some of the fundamental concepts of robotics used by all of our teams.

## Trainers

- Vivek Mhatre
- Matthew Hannay
- Nicolas Leone
- Isaac Dale
- Charlie Jenkins
- Hussain Gynai
- Kyle Stachowicz
- Kevin Fu
- Adithya Vasudev

## Meeting Schedules

TODO

## Resources

- [Software Training Mailing List](https://lists.gatech.edu/sympa/robojackets-training-sw)

  An email list for official training-related announcements.

- [Training Calendar](https://robojackets.org/calendar/action~month/cat_ids~12/request_format~html/)

  See details for all RoboJackets training meetings.

- [Piazza](https://piazza.com/class/j4pry2dzzg76pl)

  Get your questions answered any time by the instructors or other students.

- [RoboJackets Training YouTube Channel](https://www.youtube.com/channel/UCh3TLV-vQzzcWGQ4u2jsMOw)

  Where all training videos will be posted.
  
- [Software Training Repository](https://github.com/RoboJackets/software-training)

  The GitHub repository that hosts most of the resources for the software training program, including project instructions and starter code.

- [STSL Repository](https://github.com/RoboJackets/stsl)

  The Software Training Support Library repository. This holds support code for the training projects. Open issues against this repository if you find bugs with the robots or simulator.

## Prerequisites

We will assume that students are familiar with the concepts covered in [AP Computer Science A](https://apstudents.collegeboard.org/courses/ap-computer-science-a). Topics we assume knowledge of will be briefly covered in Week 0 content. Topics covered in AP Computer Science A that have special syntax, properties, or behavior in C++ will be covered in the main course content.

## Topic Schedule

The content of this program is divided into three tracks: Robotics Theory, ROS, and C++. The Robotics Theory track will survey the concepts and math that make intelligent mobile robots work. The ROS track will cover how to use the Robot Operating System to program robots. The C++ track will introduce the C++ programming language, popular in robotics applications.

Week | Robotics Theory | ROS | C++
--- | --- | --- | ---
0 | | | AP CS Review
1 | Linear Algebra, Sensors, Coordinate Frames | Introduction to ROS and useful tools | Introduction to C++
2 | Computer Vision | rclcpp Basics, Timers, Topics | Classes, Inheritance, std::bind
3 | Probability, Particle Filters | Launch, Parameters | Lifetime, References, Pointers
4 | Optimization | Services | Parallelism
5 | SLAM, Mapping | TF, Custom Interfaces | Lambdas
6 | Kalman Filters | Quality of Service | Templates
7 | Control | Actions | 
8 | Path Planning | rosbag | Iterators, Algorithms

## Project Schedule

Each week will culminate in a programming project that uses the tools and techniques covered. The projects build on each other to produce the complete software stack needed to get a custom robot to execute a challenge game.

Week | Title |  | Description
--- | --- | --- | ---
1 | Coordinate Frame Transforms | [Instructions]() | Transform fiducial detections from the camera's frame to the robot's body frame.
2 | Color-baesd Obstacle Detection | [Instructions]() | Use HSV color detection and a projective homography to find obstacles near the robot.
3 | Particle Filter Localization | [Instructions]() | Use a particle filter to localize the robot based on fiducial detections.
4 | Gradient Descent Optimization | [Instructions]() | Use gradient ascent to guide the robot to the highest simulated elevation on the map.
5 | Mapping | [Instructions]() | Build a map of the environment with a probablistic occupancy grid.
6 | Kalman Filter Tracking | [Instructions]() | Track mineral deposits with a kalman filter.
7 | LQR Controller | [Instructions]() | Control the robot's motion with an LQR controller.
8 | A-Star Path Planning | [Instructions]() | Teach the robot how to avoid obstacles with the A-Star path planning algorithm.
