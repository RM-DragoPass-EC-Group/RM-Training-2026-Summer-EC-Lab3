/**
 * @file drv_tb6612.h
 * @brief TB6612 driver migrated from the old single-motor demo project.
 */

#ifndef __DRV_TB6612_H__
#define __DRV_TB6612_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TB6612_MAX_RPM 282.0f
#define TB6612_RPM_DIR 1.0f

typedef enum
{
    TB6612_CH_FL = 0,
    TB6612_CH_FR,
    TB6612_CH_RL,
    TB6612_CH_RR,
    TB6612_CH_NUM
} TB6612_Channel_t;

typedef enum
{
    TB6612_DIR_STOP = 0,
    TB6612_DIR_FORWARD,
    TB6612_DIR_REVERSE,
    TB6612_DIR_BRAKE
} TB6612_Dir_t;

void TB6612_Init(void);
void TB6612_SetDirection(TB6612_Channel_t channel, TB6612_Dir_t dir);
void TB6612_SetDuty(TB6612_Channel_t channel, uint32_t duty);
void TB6612_Stop(TB6612_Channel_t channel);
void TB6612_Brake(TB6612_Channel_t channel);
void TB6612_StopAll(void);
void TB6612_SetRPM(TB6612_Channel_t channel, float rpm);
float TB6612_GetTargetRPM(TB6612_Channel_t channel);
uint32_t TB6612_GetMaxDuty(void);

#ifdef __cplusplus
}
#endif

#endif
