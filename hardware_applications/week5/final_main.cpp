#include <STSL/RJRobot.h>

using namespace std;

/* The DifferentialDrive class represents a differential drive robot,
 * exposing semantically meaningful motion control methods.
 */
class DifferentialDrive {
// public allows all other code to call these methods
public:
    // A simple constructor which copies the shared pointer to the robot object for use later
    DifferentialDrive(std::shared_ptr<RJRobot> robot) {
        this->robot = robot;
    }

    /*
     * A series of simple, semantically meaningful methods for driving a differential drive robot
     */

    void DriveForward() {
        robot->SetMotor(MotorPort::A, 200);
        robot->SetMotor(MotorPort::B, 200);
    }

    void DriveBackward() {
        robot->SetMotor(MotorPort::A, -200);
        robot->SetMotor(MotorPort::A, -200);
    }

    void TurnLeft() {
        robot->SetMotor(MotorPort::A, 200);
        robot->SetMotor(MotorPort::B, -200);
    }

    void TurnRight() {
        robot->SetMotor(MotorPort::A, -200);
        robot->SetMotor(MotorPort::B, 200);
    }

// private allows us to hide the behind-the-scenes details of how our class works
private:
    std::shared_ptr<RJRobot> robot;

};

int main() {

    // make_shared<> allows us to build a new RJRobot on the heap and immediately get a shared_ptr to that object.
    auto robot = std::make_shared<RJRobot>();

    // Declaring a DifferentialDrive object calls its constructor, in this case with the robot we just connected to.
    DifferentialDrive driver(robot);

    // Now we can use our simple drive methods to drive in a square!
    for(auto i = 0; i < 4; i++) {
        driver.DriveForward();
        robot->Wait(500ms);
        driver.TurnRight();
        robot->Wait(500ms);
    }

    return 0;
}
