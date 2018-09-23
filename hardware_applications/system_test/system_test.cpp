#include "STSL/RJRobot.h"
#include <iostream>

using namespace std;

int main() {

    RJRobot robot(RobotType::REAL);

    robot.SetDriveMotors(127, 127);
    robot.Wait(1000ms);
    robot.StopMotors();

    robot.Wait(1000ms);

    robot.SetLiftMotor(127);
    robot.Wait(1000ms);
    robot.SetLiftMotor(0);

    robot.Wait(1000ms);

    while (true) {
        cout << "center line " << robot.GetLineValue(LineSensor::CENTER) << endl;
        robot.Wait(500ms);

        cout << "offset line " << robot.GetLineValue(LineSensor::OFFSET) << endl;
        robot.Wait(500ms);

        cout << "forward color ";
        switch (robot.GetColor()) {
            case Color::RED:     cout << "RED"; break;
            case Color::BLUE:    cout << "BLUE"; break;
            case Color::UNKNOWN: cout << "UNKNOWN"; break;
            default: cout << "error";
        }
        cout << endl;
        robot.Wait(500ms);

        cout << "gesture ";
        switch (robot.GetGesture()) {
            case Gesture::RIGHT:  cout << "RIGHT"; break;
            case Gesture::LEFT:   cout << "BLUE"; break;
            case Gesture::UP:     cout << "UP"; break;
            case Gesture::DOWN:   cout << "DOWN"; break;
            case Gesture::NONE:   cout << "RIGHT"; break;
            default: cout << "error";
        }
        cout << endl;
        robot.Wait(500ms);

        cout << "proximity " << robot.GetProximity() << endl;
        robot.Wait(500ms);

        cout << "proximity " << robot.GetUltrasonicDistance() << endl;
        robot.Wait(500ms);
    }

    return 0;
}
