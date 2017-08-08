#ifndef TRAININGSUPPORTLIBRARY_LINUXUTILS_H
#define TRAININGSUPPORTLIBRARY_LINUXUTILS_H

#include "OSUtils.h"
#include <vector>

class LinuxUtils : public OSUtils {
public:
    virtual std::string FindRobot() override;

    virtual void Sleep(std::chrono::microseconds duration) override;

protected:

    std::vector<std::string> FindConnectedArduinos();

};


#endif //TRAININGSUPPORTLIBRARY_LINUXUTILS_H
