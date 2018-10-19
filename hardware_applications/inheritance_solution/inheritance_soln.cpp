#include "RJRobotExtended_solution.h"

using namespace std;

int main() {
    RJRobotExtended robot(RobotType::REAL, 1000ms);

    robot.DriveInSquare();

    return 0;
}
