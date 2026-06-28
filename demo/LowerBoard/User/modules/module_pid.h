/**
 * @file module_pid.h
 * @brief CMSIS-DSP PID wrapper used by application code.
 */

#ifndef __MODULE_PID_H__
#define __MODULE_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "dsp/controller_functions.h"

typedef struct
{
    arm_pid_instance_f32 dsp_pid;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;

    float setpoint;
    float feedback;
    float error;
    float last_error;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_SetParam(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_t *pid, float output_min, float output_max);
void PID_SetIntegralLimit(PID_t *pid, float integral_min, float integral_max);
void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float setpoint, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
