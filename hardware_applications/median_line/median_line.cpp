#include <STSL/RJRobot.h>
#include <vector>
#include <algorithm>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    // use these and adjust them if you need to
    const int kBlackThresh = 2000;
    const auto kSamplePeriod = 50ms;

    // make a vector of type int named line_samples

    // start driving forward

    // record light sensor samples until you sense a black line, then stop

    // find the median element's value without messing up the order of line_samples

    // find the number of timesteps in between the sample with the median value and the end of the samples

    // calculate how much time you need to back up, and back up for that long
}
