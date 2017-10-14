#include <iostream>
#include <STSL/RJRobot.h>
#include <vector>
#include <algorithm>

using namespace std;

int main() {

    RJRobot robot;

    vector<int> color_values;
    int black_thresh = 53;

    robot.SetMotor(MotorPort::A, 250);
    robot.SetMotor(MotorPort::B, 175);

    while (color_values.empty() || robot.LightValue() > black_thresh) {
//        cout << robot.LightValue() << endl;
        color_values.push_back(robot.LightValue());
        robot.Wait(250ms);
    }

    robot.SetMotor(MotorPort::A, 0);
    robot.SetMotor(MotorPort::B, 0);

    color_values.pop_back();

    auto is_grey = bind2nd(less<int>{}, 90);

    auto number_of_grey_square = count_if(color_values.begin(), color_values.end(), is_grey);

    cout << number_of_grey_square << endl;

    return 0;
}