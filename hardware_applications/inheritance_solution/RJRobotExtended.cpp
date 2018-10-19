#include "RJRobotExtended_solution.h"

using namespace std::chrono_literals;

RJRobotExtended::RJRobotExtended(RobotType type, std::chrono::milliseconds duration_in) : RJRobot(type) {
    duration = duration_in;
}

void RJRobotExtended::DriveInSquare() {
    for (int i = 0; i < 4; i++) {
        DriveEdge();
        DriveCorner();
    }
}


void RJRobotExtended::DriveEdge() {
    SetDriveMotors(120, 120);
    Wait(duration);
    StopMotors();
}
void RJRobotExtended::DriveCorner() {
    SetDriveMotors(75, -75);
    Wait(400ms); //This number might be very wrong
}

