#include <STSL/RJRobot.h>
#include <vector>
#include <algorithm>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    // use these and adjust them if you need to
    const int kBlackThresh = 2000;
    const auto kSamplePeriod = 50ms;  // the actual type here is a std::chrono::duration, but we don't need to care

    vector<int> line_samples;

    robot.SetDriveMotors(70, 70);

    int value;
    do {
        value = robot.GetLineValue(LineSensor::CENTER);
        line_samples.push_back(value);
        robot.Wait(kSamplePeriod);
    } while (value < kBlackThresh);

    robot.StopMotors();

    // find the median element's value without messing up the order of line_samples
    vector<int> samples_copy(line_samples);
    long median_index = line_samples.size() / 2;
    nth_element(samples_copy.begin(), samples_copy.end(), samples_copy.begin() + median_index);
    int median_value = samples_copy[median_index];

    // find the number of timesteps from the median-valued sample to the end of the samples as they were recorded
    auto median_iterator = find(line_samples.begin(), line_samples.end(), median_value);
    long n_timesteps = line_samples.end() - median_iterator;
    auto reverse_time = kSamplePeriod * n_timesteps;

    robot.SetDriveMotors(70, 70);
    robot.Wait(reverse_time);
    robot.StopMotors();
}
