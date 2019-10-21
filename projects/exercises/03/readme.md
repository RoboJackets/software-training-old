# Exercise 03

## Concept

We will be designing a PID controller to make the robot travel at constant velocity. What is a PID controller? A PID controller is specific method for "controlling" a system. In other words, the idea is to make the physical system match the target value. This can be in terms of position, velocity, acceleration, altitude, pressure, etc... any thing that can be measured and adjusted. 

The PID controller is an algoirthm that minimizes the error between the target and the measured value. It takes the form of:

output = (kp * error) + ki * integral(error) + kd * derivative(error)

Where the error is the difference between the target velocity and the measured velocity. kp, ki, and kd are what we call "gains" they are constants that we need to tune to get the proper response of your system. i.e. to make the ouput match the target. So what do we do after we have the "ouput"? We need to apply that to the motors. 

The motors can only take an input between -1 and 1, so you can imagine that if you make any of your gains high, you will easily go out of that range. When a system is limited in terms of its input like this, we call it saturation. We should make sure that our pid output is saturated to below 1 and above -1. This will prevent us from passing an invalid number into our motor outputs.

If we want the robot to travel in a straight line, then we want both wheel velocities to be the same. This means that you will need two different wheel velocity PID controllers: one for the right wheel and one for the left wheel.

## Running this code

1. https://find-robot.robojackets.org
1. As always, make sure you run the code using the "C++ (RoboJackets)" runner
1. Simply use the run button

Please Note:
This guide will give you guidelines on how to complete the task, but when working with hardware you should always test often and with small changes. This will make it easier to debug the code. When you make lots of changes, and especially when you introduce new hardware elements, it can be hard to tell what is malfunctioning.

## 1. Implement the PID Controller Class

The pid.h file is given to you. This file provides the variables you can use (although feel free to change them as you see fit), and the member function prototypes. You will need to implement the details of the member functions. Do not worry about the details of the class. You can treat those member functions mostly like normal functions. We will cover classes in more depth next week.

## 2. Write the code to use the PID Controller Class

Here we will need to use the PID class. You will need to instantiate the class (possibly multiple times) then use the member functions appropriately. A PID controller will need inputs from the physical world, so you will need to get the encoder readings from the robot. Think back to the concept section to determine exaclty what inputs you will need.

## 3. Tuning

Part of making a PID controller is tuning the kp, ki, and kd gains in the controller. Rather than just guessing and checking (although completly valid), you can attempt to think about what order of magnitude will make the most sense. For example, if the drive command only takes in inputs between -1 and 1, would it make sense if your kp gain was 100000? Probably not, that would be really large.

## 4. Add advanced functionality to the PID Class

Remember that an integrator constatnly sums the error. This can be an issue if there is saturation in your system. What can happen is you are apply maximum effort, but do to saturation you are still not adding enough effort. So your integrator will contiue to grow and grow, otherwise known as "integrator windup". So we can use something call antiwindup. There are many implementations, but the main idea is that we never let the integrator get to big - we never let it "windup". One simple implementation is to just stop summing the integrator if we are over a certain threshold and subtract a small amount from it to get it below the threshold. Attempt to impement this in the PID controller.

## Bonus:

Make the PID controller distance based rather than velocity based. This will require you to convert the encoder velocities to positions through integration. Good Luck!
