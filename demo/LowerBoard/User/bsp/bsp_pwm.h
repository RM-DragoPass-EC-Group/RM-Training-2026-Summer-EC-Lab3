/**
 * @file bsp_pwm.h
 * @brief PWM BSP migrated from the old TB6612 demo project.
 */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
    PWM_CH_FL = 0,
    PWM_CH_FR,
    PWM_CH_RL,
    PWM_CH_RR,
    PWM_CH_NUM
} PWM_Channel_t;

HAL_StatusTypeDef PWM_Init(void);
void PWM_SetDuty(PWM_Channel_t channel, uint32_t duty);
uint32_t PWM_GetMaxDuty(void);

/* Legacy single-motor API kept for migration teaching. */
void setPwmDuty(uint32_t duty);

#ifdef __cplusplus
}
#endif

#endif
