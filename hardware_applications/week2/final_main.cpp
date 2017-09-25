#include <iostream>
#include <STSL/RJRobot.h>
#include "final_helpers.h"

using namespace std;

int main() {

    RJRobot robot;

    cout << "Robot ready!" << endl;

    string commands = "forward back left right left forward waitForButton stop";

    executeCommandString(robot, commands);

    return 0;
}

