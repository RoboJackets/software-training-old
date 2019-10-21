#include <STSL/RJRobot.h>
#include <cstdlib>

class PID
{
private:
    float kp;
    float ki;
    float kd;
    
    float prev_error;
    float integrator;
    
    float saturation;
    float antiwindup;
    
    float dt;
    
public:
    PID(float p, float i, float d, float dt, float saturation, float antiwindup);
    float update(float target, float current);
};