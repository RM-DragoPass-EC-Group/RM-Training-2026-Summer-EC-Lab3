#include "bsp_pwm.h"

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
} PWM_Config_t;

static const PWM_Config_t pwm_config[PWM_CH_NUM] = {
    [PWM_CH_FL] = {&htim5, TIM_CHANNEL_1},
    [PWM_CH_FR] = {&htim5, TIM_CHANNEL_2},
    [PWM_CH_RL] = {&htim5, TIM_CHANNEL_4},
    [PWM_CH_RR] = {&htim5, TIM_CHANNEL_3},
};

static uint8_t PWM_IsValidChannel(PWM_Channel_t channel)
{
    return channel < PWM_CH_NUM;
}

HAL_StatusTypeDef PWM_Init(void)
{
    for (uint8_t i = 0u; i < PWM_CH_NUM; i++)
    {
        if (HAL_TIM_PWM_Start(pwm_config[i].htim, pwm_config[i].tim_channel) != HAL_OK)
        {
            return HAL_ERROR;
        }

        __HAL_TIM_SET_COMPARE(pwm_config[i].htim, pwm_config[i].tim_channel, 0u);
    }

    return HAL_OK;
}

void PWM_SetDuty(PWM_Channel_t channel, uint32_t duty)
{
    uint32_t max_duty = PWM_GetMaxDuty();

    if (!PWM_IsValidChannel(channel))
    {
        return;
    }

    if (duty > max_duty)
    {
        duty = max_duty;
    }

    __HAL_TIM_SET_COMPARE(pwm_config[channel].htim, pwm_config[channel].tim_channel, duty);
}

uint32_t PWM_GetMaxDuty(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim5);
}

void setPwmDuty(uint32_t duty)
{
    PWM_SetDuty(PWM_CH_FL, duty);
}
