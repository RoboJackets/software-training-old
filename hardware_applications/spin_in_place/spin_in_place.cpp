#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot(RobotType::REAL);

    while (true) {
        robot.SetDriveMotors(200, -200);
        robot.Wait(1000ms);
        robot.StopMotors();
        robot.Wait(1000ms);
    }

    return 0;
}
