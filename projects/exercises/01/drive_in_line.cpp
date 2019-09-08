#include <STSL/RJRobot.h>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL); //Initialize a RJ Robot that controls a real robot
    
    robot.setDriveMotors(127, 127); //Drive both robot motors forward at max speed
    robot.Wait(2000ms); //Wait for 2000 milliseconds (2 seconds)
    robot.StopMotors(); //Stop the motors

}
