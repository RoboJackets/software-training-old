#include "STSL/RJRobot.h"


class RJRobotExtended : public RJRobot {
public:
    RJRobotExtended(RobotType type, std::chrono::milliseconds duration_in);
    void DriveInSquare();

private:
    std::chrono::milliseconds duration;
    void DriveEdge();
    void DriveCorner();
};