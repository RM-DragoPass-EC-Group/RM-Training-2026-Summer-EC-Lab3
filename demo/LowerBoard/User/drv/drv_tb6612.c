#include "drv_tb6612.h"

#include "bsp/bsp_pwm.h"

typedef struct
{
    GPIO_TypeDef *out1_port;
    uint16_t out1_pin;
    GPIO_TypeDef *out2_port;
    uint16_t out2_pin;
    PWM_Channel_t pwm_channel;
    float target_rpm;
} TB6612_Config_t;

static TB6612_Config_t tb6612[TB6612_CH_NUM] = {
    [TB6612_CH_FL] = {FL_OUT1_GPIO_Port, FL_OUT1_Pin, FL_OUT2_GPIO_Port, FL_OUT2_Pin, PWM_CH_FL, 0.0f},
    [TB6612_CH_FR] = {FR_OUT1_GPIO_Port, FR_OUT1_Pin, FR_OUT2_GPIO_Port, FR_OUT2_Pin, PWM_CH_FR, 0.0f},
    [TB6612_CH_RL] = {RL_OUT1_GPIO_Port, RL_OUT1_Pin, RL_OUT2_GPIO_Port, RL_OUT2_Pin, PWM_CH_RL, 0.0f},
    [TB6612_CH_RR] = {RR_OUT1_GPIO_Port, RR_OUT1_Pin, RR_OUT2_GPIO_Port, RR_OUT2_Pin, PWM_CH_RR, 0.0f},
};

static uint8_t TB6612_IsValidChannel(TB6612_Channel_t channel)
{
    return channel < TB6612_CH_NUM;
}

static float TB6612_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

void TB6612_Init(void)
{
    PWM_Init();
    TB6612_StopAll();
}

void TB6612_SetDirection(TB6612_Channel_t channel, TB6612_Dir_t dir)
{
    if (!TB6612_IsValidChannel(channel))
    {
        return;
    }

    switch (dir)
    {
        case TB6612_DIR_FORWARD:
            HAL_GPIO_WritePin(tb6612[channel].out1_port, tb6612[channel].out1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(tb6612[channel].out2_port, tb6612[channel].out2_pin, GPIO_PIN_RESET);
            break;

        case TB6612_DIR_REVERSE:
            HAL_GPIO_WritePin(tb6612[channel].out1_port, tb6612[channel].out1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tb6612[channel].out2_port, tb6612[channel].out2_pin, GPIO_PIN_SET);
            break;

        case TB6612_DIR_BRAKE:
            HAL_GPIO_WritePin(tb6612[channel].out1_port, tb6612[channel].out1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(tb6612[channel].out2_port, tb6612[channel].out2_pin, GPIO_PIN_SET);
            break;

        case TB6612_DIR_STOP:
        default:
            HAL_GPIO_WritePin(tb6612[channel].out1_port, tb6612[channel].out1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(tb6612[channel].out2_port, tb6612[channel].out2_pin, GPIO_PIN_RESET);
            break;
    }
}

void TB6612_SetDuty(TB6612_Channel_t channel, uint32_t duty)
{
    if (!TB6612_IsValidChannel(channel))
    {
        return;
    }

    PWM_SetDuty(tb6612[channel].pwm_channel, duty);
}

void TB6612_Stop(TB6612_Channel_t channel)
{
    if (!TB6612_IsValidChannel(channel))
    {
        return;
    }

    tb6612[channel].target_rpm = 0.0f;
    TB6612_SetDuty(channel, 0u);
    TB6612_SetDirection(channel, TB6612_DIR_STOP);
}

void TB6612_Brake(TB6612_Channel_t channel)
{
    if (!TB6612_IsValidChannel(channel))
    {
        return;
    }

    tb6612[channel].target_rpm = 0.0f;
    TB6612_SetDuty(channel, TB6612_GetMaxDuty());
    TB6612_SetDirection(channel, TB6612_DIR_BRAKE);
}

void TB6612_StopAll(void)
{
    for (uint8_t i = 0u; i < TB6612_CH_NUM; i++)
    {
        TB6612_Stop((TB6612_Channel_t)i);
    }
}

void TB6612_SetRPM(TB6612_Channel_t channel, float rpm)
{
    float motor_rpm;
    float abs_rpm;
    uint32_t duty;

    if (!TB6612_IsValidChannel(channel))
    {
        return;
    }

    motor_rpm = rpm * TB6612_RPM_DIR;
    abs_rpm = TB6612_AbsFloat(motor_rpm);
    duty = 0u;

    tb6612[channel].target_rpm = rpm;

    if (abs_rpm <= 0.0f)
    {
        TB6612_Stop(channel);
        return;
    }

    if (abs_rpm > TB6612_MAX_RPM)
    {
        abs_rpm = TB6612_MAX_RPM;
    }

    duty = (uint32_t)((abs_rpm / TB6612_MAX_RPM) * (float)TB6612_GetMaxDuty());

    if (motor_rpm > 0.0f)
    {
        TB6612_SetDirection(channel, TB6612_DIR_FORWARD);
    }
    else
    {
        TB6612_SetDirection(channel, TB6612_DIR_REVERSE);
    }

    TB6612_SetDuty(channel, duty);
}

float TB6612_GetTargetRPM(TB6612_Channel_t channel)
{
    if (!TB6612_IsValidChannel(channel))
    {
        return 0.0f;
    }

    return tb6612[channel].target_rpm;
}

uint32_t TB6612_GetMaxDuty(void)
{
    return PWM_GetMaxDuty();
}
