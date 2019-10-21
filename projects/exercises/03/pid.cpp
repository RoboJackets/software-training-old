#include "pid.h"

PID::PID(float p, float i, float d, float dt)
{
    kp = p;
    ki = i;
    kd = d;
    
    prev_error = 0;
    integrator = 0;
    
    this->dt = dt;
}

float PID::update(float target, float current)
{
    float error = target - current;
    integrator += error * dt;
    float derivative = (error - prev_error) / dt;
    float output = (kp * error) + (ki * integrator) + (kd * derivative);
    prev_error = error;
    return output;
}