#include <STSL/RJRobot.h>

using namespace std;
using namespace std::chrono_literals;  // allows you to use times like 1000ms, 1s, etc.

int main()
{
    RJRobot robot(RobotType::REAL);

    for (int i = 0; i < 4; i++) {
        robot.SetMotor(Motor::LEFT, 127);
        robot.SetMotor(Motor::RIGHT, 127);
        robot.Wait(1000ms);
        robot.StopMotors();

        robot.SetMotor(Motor::LEFT, 127);
        robot.Wait(1000ms);
        robot.StopMotors();
    }
}
