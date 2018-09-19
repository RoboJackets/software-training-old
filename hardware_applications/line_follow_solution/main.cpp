#include <STSL/RJRobot.h>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    while (true) {
        if (robot.GetLineValue(LineSensor::CENTER) < 127) {
            robot.SetMotor(Motor::LEFT, 127);
            robot.SetMotor(Motor::RIGHT, 0);
        } else {
            robot.SetMotor(Motor::RIGHT, 127);
            robot.SetMotor(Motor::LEFT, 0);
        }
        robot.Wait(10ms);
    }
}
