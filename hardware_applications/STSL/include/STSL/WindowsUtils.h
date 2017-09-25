#ifndef TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H
#define TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H

#include "OSUtils.h"
#include <vector>

class WindowsUtils : public OSUtils {
public:
    virtual std::string FindRobot() override;

    virtual void Sleep(std::chrono::microseconds duration) override;

protected:

    std::vector<std::string> AvailableComPorts();
};


#endif //TRAININGSUPPORTLIBRARY_WINDOWSUTILS_H
