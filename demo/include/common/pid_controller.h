#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;

    float setpoint;
    float feedback;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_SetParam(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_t *pid, float output_min, float output_max);
void PID_SetIntegralLimit(PID_t *pid, float integral_min, float integral_max);
void PID_SetDeadband(PID_t *pid, float deadband);
void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float setpoint, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif
