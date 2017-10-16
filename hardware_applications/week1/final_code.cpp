#include <iostream>
#include <STSL/RJRobot.h>

using namespace std;

int main() {
    RJRobot robot;

    cout << "Robot ready!" << endl;

    robot.SetMotor(MotorPort::B, 200);

    for(int i = 0; i < 4; i++) {
        robot.SetMotor(MotorPort::A, 200);
        robot.Wait(1000ms);
        robot.SetMotor(MotorPort::A, -200);
        robot.Wait(1300ms);
    }

    robot.StopMotors();

    robot.Wait(1s);

    robot.SetMotor(MotorPort::A, 200);
    robot.SetMotor(MotorPort::B, 200);

    while(!robot.IsButtonPressed()) {
        robot.Wait(50ms);
    }

    robot.StopMotors();

    return 0;
}