#include "pid.h"

PID::PID(float p, float i, float d, float dt_in, float saturation_in, float antiwindup_in)
{
    kp = p;
    ki = i;
    kd = d;
    
    prev_error = 0;
    integrator = 0;
    
    dt = dt_in;
    saturation = saturation_in;
    antiwindup = antiwindup_in;
    
}

float PID::update(float target, float current)
{
    float error = target - current;
    
    
    if (integrator < antiwindup)
    {
        integrator += error * dt;
    }

    float derivative = (error - prev_error) / dt;
    
    float output = (kp * error) + (ki * integrator) + (kd * derivative);
    prev_error = error;
    
    if (std::abs(output) > saturation) // saturate
    {
        std::copysign(saturation, output);
    }
    return output;
}