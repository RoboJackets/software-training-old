#include <iostream>
#include <STSL/RJRobot.h>
#include <vector>
#include <algorithm>

using namespace std;

int main() {

    RJRobot robot;

    /*
     * Count the number of grey squares on a strip of paper
     */

    vector<int> measurements;

    auto black_threshold = 10;

    auto grey_threshold = 40;

    // Measure light values until we see a black square
    while(measurements.empty() || measurements.back() > black_threshold) {
        // Measure square color
        measurements.push_back(robot.LightValue());

        // Move to next square
        robot.SetMotor(MotorPort::A, 200);
        robot.SetMotor(MotorPort::B, 200);
        robot.Wait(250ms);
    }

    // Remove the black square from the list
    measurements.erase(measurements.end()-1);

    // comment here
    auto is_grey = bind2nd(less<int>{}, grey_threshold);

    // comment here
    auto number_of_grey_squares = count_if(measurements.begin(), measurements.end(), is_grey);

    cout << number_of_grey_squares << " grey squares detected." << endl;

    return 0;
}
