#include <STSL/RJRobot.h>

int main()
{
	RJRobot robot;

	robot.setDriveMotors(1.0, 0); //Left on
	robot.wait(1000ms);

	robot.setDriveMotors(0, 1.0); //Right on, left off
	robot.wait(1000ms);

	robot.stopMotors();

	std::cout << robot.getBatteryVoltage() << std::endl;
	std::cout << robot.getCenterLineSensor() << std::endl;
	std::cout << robot.getOffsetLineSensor() << std::endl;
}