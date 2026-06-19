#include "common_pid.h"
#include <stdint.h>


void Common_PID_Calculate(PID_Controller *pid)
{
    pid->error = pid->measurement - pid->target;
    pid->integral += pid->error;
    if(pid->last_error == 0)
    {
        pid->last_error = pid->error;
    }
    float der = pid->error - pid->last_error;

    pid->output = (pid->Kp * pid->error) + (pid->Ki * pid->integral * PID_TIME) + (pid->Kd * der / PID_TIME);

    pid->last_error = pid->error;
}

void Common_PID_Calculate_chain(PID_Controller *out_pid, PID_Controller *in_pid)
{
    Common_PID_Calculate(out_pid);
    in_pid->target = out_pid->output;
    Common_PID_Calculate(in_pid);
}

int16_t Common_Limit(int16_t value, int16_t min, int16_t max)
{
    if(value > max)
    {
        return max;
    }
    else if(value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}
