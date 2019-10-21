#include <STSL/RJRobot.h>
#include "pid.h"
#include <ctime>
#include <unistd.h>
#include <cstdlib>

int main()
{
    PID controller_left(10.0, 0.1, 0, .01, 0.5, 0.25);
    PID controller_right(10.0, 0.1, 0, .01, 0.5, 0.25);
    
    float target = 0.5;
    float current = 0;
    float controller_effort_left, controller_effort_right;
    int dt = 10;
    
    RJRobot robot;
    RJRobot::EncoderSpeeds speeds;
    
    std::cout << "Starting Loop" << std::endl;
    
    while(std::abs(target - current) > 0.001)
    {
        std::cout << "In loop" << std::endl;
        
        auto start_time = std::chrono::system_clock::now();
        
        speeds = robot.getEncoderSpeeds(); // Get the encoder speeds
        controller_effort_left = controller_left.update(target, speeds.left); // PID on left wheel
        controller_effort_right = controller_right.update(target, speeds.right); // PID on right wheel
        robot.setDriveMotors(controller_effort_left, -1*controller_effort_right); // Set drive motors
        
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time-start_time; // Can use elapsed time for more accurate dt
        
        robot.wait(std::chrono::duration_cast<std::chrono::microseconds>(operator""ms(dt))); 
        std::cout << "Left Speed: " << speeds.left << std::endl;
        std::cout << " Right Speed: " << speeds.right << std::endl;
    
        current = (speeds.left + speeds.right)/2; // Average the two speeds to get the current velocity
        
    }
    
    /*robot.setDriveMotors(0.375, -0.375);
    robot.wait(std::chrono::duration_cast<std::chrono::microseconds>(operator""ms(10000)));*/
    robot.stopMotors();
        
    return 0;
}