#include <STSL/RJRobot.h>

class PID
{
private:
    float kp;
    float ki;
    float kd;
    
    float prev_error;
    float integrator;
    
    float dt;
    
public:
    PID(float p, float i, float d, float dt);
    float update(float target, float current);
};