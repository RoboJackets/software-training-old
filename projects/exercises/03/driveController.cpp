#include <STSL/RJRobot.h>
#include "pid.h"
#include <ctime>
#include <unistd.h>

int main()
{
    PID controller(1.0, 0, 0, .01);
    
    float target = 1;
    float current = 0;
    float controller_effort;
    
    RJRobot robot;
    
    auto prev_time = std::chrono::system_clock::now();
    
    //robot.wait(1000);
    //std::this_thread::sleep_for (std::chrono::seconds(1));
   // usleep(10000);

    auto curr_time = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = curr_time-prev_time;


    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";    
    
    RJRobot::EncoderSpeeds speeds;
    
    //speeds = robot.getEncoderSpeeds();
    controller_effort = controller.update(target, speeds.left);
    
    std::cout << "Controller Effort: " << controller_effort << std::endl;
    
    
    return 0;
}