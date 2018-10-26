#include <iostream>
#include <algorithm>
#include <vector>

#include <STSL/RJRobot.h>

using namespace std;

const int black_thresh = 2000;

int main() {
  RJRobot robot(RobotType::REAL);

  robot.SetDriveMotors(100, 100);

  vector<int> values;
  int value;
  do {
    value = robot.GetLineValue(LineSensor::CENTER);
    values.push_back(value);
  } while (value < black_thresh);

  robot.StopMotors();

  // Categorize every value into a bin. You can use bin(x) = x / 256 to get a bin number 0-15
  // This is easy to do without STL functions, but try to use one for practice here

  // Print out the number of readings in each bin. What STL algo does this easily?
}
