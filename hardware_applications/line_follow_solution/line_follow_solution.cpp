#include <STSL/RJRobot.h>

#include <iostream>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    int thresh = 2000;

    while (true) {
        int light_val = robot.GetLineValue(LineSensor::CENTER);
        cout << light_val << endl;

        if (light_val < thresh) {
            robot.SetDriveMotors(-30, 200);
        } else {
            robot.SetDriveMotors(200, -30);
        }
    }
}
