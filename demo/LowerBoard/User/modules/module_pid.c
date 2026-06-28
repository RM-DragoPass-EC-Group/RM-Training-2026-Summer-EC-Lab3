#include "module_pid.h"

#include <stddef.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->dsp_pid.Kp = kp;
    pid->dsp_pid.Ki = ki;
    pid->dsp_pid.Kd = kd;
    pid->output_min = -100.0f;
    pid->output_max = 100.0f;
    pid->integral_min = -100.0f;
    pid->integral_max = 100.0f;
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;

    arm_pid_init_f32(&pid->dsp_pid, 1);
}

void PID_SetParam(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->dsp_pid.Kp = kp;
    pid->dsp_pid.Ki = ki;
    pid->dsp_pid.Kd = kd;
    arm_pid_init_f32(&pid->dsp_pid, 0);
}

void PID_SetOutputLimit(PID_t *pid, float output_min, float output_max)
{
    if ((pid == NULL) || (output_min > output_max))
    {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->output = PID_Clamp(pid->output, output_min, output_max);
    pid->dsp_pid.state[2] = pid->output;
}

void PID_SetIntegralLimit(PID_t *pid, float integral_min, float integral_max)
{
    if ((pid == NULL) || (integral_min > integral_max))
    {
        return;
    }

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    arm_pid_reset_f32(&pid->dsp_pid);
}

float PID_Calc(PID_t *pid, float setpoint, float feedback, float dt_s)
{
    float output;

    (void)dt_s;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->setpoint = setpoint;
    pid->feedback = feedback;
    pid->last_error = pid->error;
    pid->error = setpoint - feedback;

    output = arm_pid_f32(&pid->dsp_pid, pid->error);
    pid->output = PID_Clamp(output, pid->output_min, pid->output_max);
    pid->dsp_pid.state[2] = pid->output;

    return pid->output;
}
