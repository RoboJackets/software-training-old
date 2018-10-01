#include <STSL/RJRobot.h>
#include <vector>
#include <algorithm>

using namespace std;

int main()
{
    RJRobot robot(RobotType::REAL);

    // use these and adjust them if you need to
    const int kBlackThresh = 2000;

    // make a vector of type int named line_samples

    // start driving forward

    // record light sensor samples until you sense a black line, then stop

    // find the median element's value (which algorithm can do this efficiently?)

    // back up until you see a value close to the median
}
