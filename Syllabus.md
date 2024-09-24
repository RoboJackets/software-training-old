# RoboJackets Software Training - Fall 2024

The RoboJackets software training program is designed to prepare you to work on software for the RoboJackets competitive teams. This program offers practical experience writing code using C++ and ROS. It also covers some of the fundamental concepts of robotics used by all of our teams.

## Trainers
- Mukilan Karthikeyan (Software Core Chair)
- Aalind Tyagi (Software Training Lead) [using software-v3](https://github.com/RoboJackets/software-training-v3)


## Meeting Schedules

All new software members will attend two meetings each week. One meeting will be with their competition team. The other is a general training meeting for members of all teams. While each competition team has its own schedule for new member meetings, there are multiple sessions of the general training meetings available. These general sessions are identical and exist just to give more flexible scheduling options.

For more information about team meeting schedules and locations, check with each team's leadership.

### New Member Team Meetings
RoboWrestling | RoboRacing | RoboNav | RoboCup
--- | --- | --- | ---
Sundays, 1:00 - 4:00pm | Sunday, 4:00 - 7:00pm | Sundays, 4:00 - 7:00pm | Sundays, 7:00 - 9:30pm
 | Mondays, 7:00 - 9:30pm | | 



### General Software Training Meetings
Modays | Fridays
--- | --- 
5:00 - 6:00pm | 5:00 - 6:00pm 
MoSE G011 | MoSE G011


## Resources

- [Software Training Mailing List](https://lists.gatech.edu/sympa/robojackets-training-sw)

  An email list for official training-related announcements.

- [Training Calendar](https://robojackets.org/calendar/action~month/cat_ids~12/request_format~html/)

  See details for all RoboJackets training meetings.

- [Piazza](https://piazza.com/class/l7889f6vhrf1nv)

  Get your questions answered any time by the instructors or other students.

- [RoboJackets Training YouTube Channel](https://www.youtube.com/channel/UCh3TLV-vQzzcWGQ4u2jsMOw)

  Where all training videos will be posted.

- [Software Training Repository](https://github.com/RoboJackets/software-training)

  The GitHub repository that hosts most of the resources for the software training program, including project instructions and starter code.

- [STSL Repository](https://github.com/RoboJackets/stsl)

  The Software Training Support Library repository. This holds support code for the training projects. Open issues against this repository if you find bugs with the robots or simulator.

## Prerequisites

We will assume that students are familiar with the concepts covered in [AP Computer Science A](https://apstudents.collegeboard.org/courses/ap-computer-science-a). Topics we assume knowledge of will be briefly covered in Week 0 content. Topics covered in AP Computer Science A that have special syntax, properties, or behavior in C++ will be covered in the main course content.

Students should also be comfortable with math at the level of [AP Calculus AB](https://apstudents.collegeboard.org/courses/ap-calculus-ab). Derivatives and integrals will show up throughout the course.

## Topic Schedule

The content of this program is divided into three tracks: Robotics Theory, ROS, and C++. The Robotics Theory track will survey the concepts and math that make intelligent mobile robots work. The ROS track will cover how to use the Robot Operating System to program robots. The C++ track will introduce the C++ programming language, popular in robotics applications.
Robotics theory videos should be watched before your team-specific meetings. C++ and ROS videos should be watched before the general training meetings. You should watch the C++ videos before the ROS videos for a given week.

[Course Introduction Video](https://youtu.be/LZ4-nDEAFcY)

Week | Robotics Theory | ROS | C++
--- | --- | --- | ---
0 | | | [AP CS Review](https://youtube.com/playlist?list=PL1R5gSylLha2AOCmSaLdDlBMug5XFNfwv)
1 | [Linear Algebra, Sensors, Coordinate Frames](https://youtube.com/playlist?list=PL1R5gSylLha2RjafLHG9lqNqZ2rzH_hdQ) | [Introduction to ROS and useful tools](https://youtube.com/playlist?list=PL1R5gSylLha0y1U3yHAkCYJXXL-GiJDwF) | [Introduction to C++](https://youtube.com/playlist?list=PL1R5gSylLha1TChL2Lkm6PQQnOPRSIpDK)
2 | [Computer Vision](https://youtube.com/playlist?list=PL1R5gSylLha0cFU3nGomLr8cIUaKun6bl) | [rclcpp Basics, Timers, Topics](https://youtube.com/playlist?list=PL1R5gSylLha0wxbvXIiNeEr12aoO_VX_8) | [Classes, Inheritance, std::bind](https://youtube.com/playlist?list=PL1R5gSylLha3KemZ2wqInhNm-db8kR88r)
3 | [Probability, Particle Filters](https://youtube.com/playlist?list=PL1R5gSylLha2ylxbALvguW15qf-mHjsGm)  | [Launch, Parameters](https://youtube.com/playlist?list=PL1R5gSylLha3YMGovXmHZGn9wVrAChkxk) | [Lifetime, References, Pointers](https://youtube.com/playlist?list=PL1R5gSylLha2BEzoEGSt-EAmx4HbvQ7RZ)
4 | [Optimization](https://youtube.com/playlist?list=PL1R5gSylLha0975HYnqN-Jq4Jx0r7LiTu) | [Services](https://youtube.com/playlist?list=PL1R5gSylLha3QucE7Smr0-YvnV70fZkoq) | [Concurrency Basics](https://youtube.com/playlist?list=PL1R5gSylLha1B3HQldnfhFu4_rVZnW55q)
5 | [SLAM, Mapping](https://www.youtube.com/watch?v=CgiVz-KMBH0&list=PL1R5gSylLha1cX02r8hiMA85vPmfSYHP_) | [TF, Custom Interfaces](https://youtube.com/playlist?list=PL1R5gSylLha2od_7P9YuSSLsKCd3vtCY7) | [Lambdas](https://youtube.com/playlist?list=PL1R5gSylLha1huMeonsTMxqU8DE7m_zWh)
6 | [Kalman Filters](https://youtube.com/playlist?list=PL1R5gSylLha0j_tmn3YhTTs90-pUFuZH9) | [Quality of Service](https://youtube.com/playlist?list=PL1R5gSylLha0IvTKCOckpL5QvVB4Hn-97) | [Templates](https://youtube.com/playlist?list=PL1R5gSylLha3kQMd1tIxDywbOWNNYaiJM)
7 | [Control](https://youtube.com/playlist?list=PL1R5gSylLha3nYaE3PTmJIon7GgIxzr_M) | [Actions](https://youtube.com/playlist?list=PL1R5gSylLha1qUf5ngWco_EnNfYsTzAUc) |
8 | [Path Planning](https://youtube.com/playlist?list=PL1R5gSylLha1epFZYz_z2BKO0sSXNPcjM) | [Bags](https://youtube.com/playlist?list=PL1R5gSylLha2i-XmvxwzfPgBKSJ6EKcF4) | [Iterators, Algorithms](https://youtube.com/playlist?list=PL1R5gSylLha1l1f8OcxXCVtnh6XmPzFzU)


<!-- Coming Soon: 
- What is Docker?
- How to use Git Effectively
- C++ practice 
  - indexing, sorting, mattrix operations -->


## Additional Resources
The following resources are great supplemental material to learn more about software for robotics applications.

[Learn Git](https://learngitbranching.js.org/?locale=en_US)
[The Linux command line](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)
[Robotics and Perception Textbook](https://www.roboticsbook.org/intro.html)
[Broader view on Controls Matlab videos ](https://www.mathworks.com/videos/tech-talks.html)
<!-- [Nvidia's Isaac ROS](https://developer.nvidia.com/isaac/ros) -->

<!-- 
Week | Robotics Theory | ROS | C++ 
--- | --- | --- | ---
0 | | | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2AOCmSaLdDlBMug5XFNfwv)
1 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2RjafLHG9lqNqZ2rzH_hdQ) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0y1U3yHAkCYJXXL-GiJDwF) |[Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1TChL2Lkm6PQQnOPRSIpDK)

2 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0cFU3nGomLr8cIUaKun6bl) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0wxbvXIiNeEr12aoO_VX_8) |[Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3KemZ2wqInhNm-db8kR88r)

3 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2ylxbALvguW15qf-mHjsGm) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3YMGovXmHZGn9wVrAChkxk)| [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2BEzoEGSt-EAmx4HbvQ7RZ)

4 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0975HYnqN-Jq4Jx0r7LiTu) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3QucE7Smr0-YvnV70fZkoq)| [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1B3HQldnfhFu4_rVZnW55q)

5 | [Playlist](https://www.youtube.com/watch?v=CgiVz-KMBH0&list=PL1R5gSylLha1cX02r8hiMA85vPmfSYHP_) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2od_7P9YuSSLsKCd3vtCY7) |[Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1huMeonsTMxqU8DE7m_zWh)

6 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0j_tmn3YhTTs90-pUFuZH9) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0IvTKCOckpL5QvVB4Hn-97)| [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3kQMd1tIxDywbOWNNYaiJM)

7 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3nYaE3PTmJIon7GgIxzr_M) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1qUf5ngWco_EnNfYsTzAUc) | No C++ videos this week 

8 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1epFZYz_z2BKO0sSXNPcjM) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2i-XmvxwzfPgBKSJ6EKcF4) |[Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1l1f8OcxXCVtnh6XmPzFzU) -->

## Project Schedule

Each week will culminate in a programming project that uses the tools and techniques covered. The projects build on each other to produce the complete software stack needed to get a custom robot to execute a challenge game.

Week | Title |  | Description
--- | --- | --- | ---
1 | Coordinate Frame Transforms | [Instructions](projects/week_1/Instructions.md) | Transform fiducial detections from the camera's frame to the robot's body frame.
2 | Color-based Obstacle Detection | [Instructions](projects/week_2/Instructions.md) | Use HSV color detection and a projective homography to find obstacles near the robot.
3 | Particle Filter Localization | [Instructions](projects/week_3/Instructions.md) | Use a particle filter to localize the robot based on fiducial detections.
4 | Gradient Descent Optimization | [Instructions](projects/week_4/Instructions.md) | Use gradient ascent to guide the robot to the highest simulated elevation on the map.
5 | Mapping | [Instructions](projects/week_5/Instructions.md) | Build a map of the environment with a probablistic occupancy grid.
6 | Kalman Filter Tracking | [Instructions](projects/week_6/Instructions.md) | Track mineral deposits with a kalman filter.
7 | LQR Controller | [Instructions](projects/week_7/Instructions.md) | Control the robot's motion with an LQR controller.
8 | A-Star Path Planning | [Instructions](projects/week_8/Instructions.md) | Teach the robot how to avoid obstacles with the A-Star path planning algorithm.
