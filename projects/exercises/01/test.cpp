#include <STSL/RJRobot.h>

int main()
{
    RJRobot robot;
    robot.setDriveMotors(0.375, -0.375);
    robot.wait(std::chrono::duration_cast<std::chrono::microseconds>(operator""ms(10000)));
}