#include <iostream>
#include "final_helpers.h"

using namespace std;

void executeCommandString(RJRobot &robot, const std::string &commands) {
    size_t currentPosition = 0;
    auto nextSpacePosition = commands.find(' ', currentPosition);
    do {
        executeOneCommand(robot, commands.substr(currentPosition, nextSpacePosition));
        currentPosition = nextSpacePosition + 1;
        nextSpacePosition = commands.find(' ', currentPosition);
    } while(nextSpacePosition != std::string::npos);
}

void executeOneCommand(RJRobot &robot, const std::string &command) {

    if(command == "forward") {
        robot.SetMotor(MotorPort::A, 100);
        robot.SetMotor(MotorPort::B, 100);
        robot.Wait(250ms);
    }
    else if(command == "left") {
        robot.SetMotor(MotorPort::A, -100);
        robot.SetMotor(MotorPort::B, 100);
        robot.Wait(250ms);
    }
    else if(command == "right") {
        robot.SetMotor(MotorPort::A, 100);
        robot.SetMotor(MotorPort::B, -100);
        robot.Wait(250ms);
    }
    else if(command == "backward") {
        robot.SetMotor(MotorPort::A, -100);
        robot.SetMotor(MotorPort::B, -100);
        robot.Wait(250ms);
    }
    else if(command == "waitForButton") {
        while (!robot.IsButtonPressed()) {
            robot.Wait(50ms);
        }
    }
    else if(command == "stop") {
        robot.StopMotors();
    }
    else if(command == "dance") {
        // ???
    }
    else {
        std::cerr << "Unkown command: " << command << std::endl;
    }
}
