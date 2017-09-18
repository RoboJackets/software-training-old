#ifndef TRAININGSUPPORTLIBRARY_OSUTILS_H
#define TRAININGSUPPORTLIBRARY_OSUTILS_H

#include <string>
#include <chrono>

class OSUtils {
public:
    virtual std::string FindRobot() = 0;

    virtual void Sleep(std::chrono::microseconds duration) = 0;
};

#endif //TRAININGSUPPORTLIBRARY_OSUTILS_H
