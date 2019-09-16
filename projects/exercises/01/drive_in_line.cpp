#include <STSL/RJRobot.h>

int main()
{
    RJRobot robot; //Initialize a RJ Robot
    
    robot.setDriveMotors(1.0, 1.0); //Drive both robot motors forward at max speed
    robot.wait(2000ms); //Wait for 2000 milliseconds (2 seconds)
    robot.stopMotors(); //Stop the motors

    return 0;
}
