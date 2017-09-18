#ifndef TRAININGSUPPORTLIBRARY_RJROBOT_H
#define TRAININGSUPPORTLIBRARY_RJROBOT_H

#include "SerialPort.h"
#include "OSUtils.h"
#include <memory>

enum class MotorPort {
    A,
    B
};

class RJRobot {

public:
    RJRobot();

    ~RJRobot();

    bool IsButtonPressed();

    uint8_t LightValue();

    void SetFloodlight(bool on);

    void SetMotor(const MotorPort &port, const int &speed);

    void StopMotors();

    void Wait(std::chrono::microseconds duration);

private:

    SerialPort serial_port_;

    std::unique_ptr<OSUtils> os_utils_;

};


#endif //TRAININGSUPPORTLIBRARY_RJROBOT_H
