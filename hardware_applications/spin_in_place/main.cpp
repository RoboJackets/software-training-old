#include "STSL/RJRobot.h"

using namespace std;

int main() {

    RJRobot robot(RobotType::REAL);

    while(true) {
        robot.SetMotor(Motor::LEFT, -127);
        robot.SetMotor(Motor::RIGHT, 127);
        robot.Wait(1000ms);
        robot.StopMotors();
        robot.Wait(1000ms);
    }

    return 0;
}
