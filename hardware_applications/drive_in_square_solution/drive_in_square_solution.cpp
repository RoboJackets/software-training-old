#include <STSL/RJRobot.h>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    for (int i = 0; i < 4; i++) {
        robot.SetDriveMotors(127, 127);
        robot.Wait(1000ms);
        robot.StopMotors();

        robot.SetDriveMotors(127, -30);
        robot.Wait(500ms);
        robot.StopMotors();
    }
}
