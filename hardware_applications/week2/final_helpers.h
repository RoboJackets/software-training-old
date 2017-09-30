#ifndef HARDWARE_APPLICATIONS_FINAL_CODE_H
#define HARDWARE_APPLICATIONS_FINAL_CODE_H

#include <STSL/RJRobot.h>

void executeCommandString(RJRobot &robot, const std::string &commands);

void executeOneCommand(RJRobot &robot, const std::string &command);

#endif //HARDWARE_APPLICATIONS_FINAL_CODE_H
