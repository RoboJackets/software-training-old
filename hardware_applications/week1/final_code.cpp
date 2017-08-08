#include <iostream>
#include <STSL/RJRobot.h>

using namespace std;

int main() {
    RJRobot robot;

    cout << "Robot ready!" << endl;

    robot.SetMotor(MotorPort::B, 100);

    for(int i = 0; i < 4; i++) {
        robot.SetMotor(MotorPort::A, 100);
        robot.Wait(500ms);
        robot.SetMotor(MotorPort::A, -100);
        robot.Wait(250ms);
    }

    robot.StopMotors();

    robot.Wait(1s);

    robot.SetMotor(MotorPort::A, 100);
    robot.SetMotor(MotorPort::B, 100);

    while(!robot.IsButtonPressed()) {
        robot.Wait(50ms);
    }

    robot.StopMotors();

    return 0;
}