#include <STSL/RJRobot.h>
#include "pid.h"
#include <ctime>
#include <unistd.h>
#include <cstdlib>

int main()
{
    // Initialize PID objects
    
    // Initialzie variables
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
        

        // Get the encoder speeds
        // PID on left wheel
        // PID on right wheel
        // Set drive motors
        
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