#include "bsp_timer.h"

#include "tim.h"

static volatile uint32_t bsp_timer_1khz_tick = 0u;
static volatile uint32_t bsp_timer_100hz_tick = 0u;
static BSP_Timer1kHzCallback_t bsp_timer_1khz_callback = NULL;
static BSP_Timer100HzCallback_t bsp_timer_100hz_callback = NULL;

HAL_StatusTypeDef BSP_Timer1kHz_Init(void)
{
    HAL_TIM_Base_Stop_IT(&htim10);
    bsp_timer_1khz_tick = 0u;
    __HAL_TIM_SET_COUNTER(&htim10, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    return HAL_TIM_Base_Start_IT(&htim10);
}

HAL_StatusTypeDef BSP_Timer100Hz_Init(void)
{
    HAL_TIM_Base_Stop_IT(&htim11);
    bsp_timer_100hz_tick = 0u;
    __HAL_TIM_SET_COUNTER(&htim11, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);

    return HAL_TIM_Base_Start_IT(&htim11);
}

void BSP_Timer1kHz_RegisterCallback(BSP_Timer1kHzCallback_t callback)
{
    bsp_timer_1khz_callback = callback;
}

void BSP_Timer100Hz_RegisterCallback(BSP_Timer100HzCallback_t callback)
{
    bsp_timer_100hz_callback = callback;
}

HAL_StatusTypeDef BSP_Timer1kHz_Stop(void)
{
    return HAL_TIM_Base_Stop_IT(&htim10);
}

HAL_StatusTypeDef BSP_Timer100Hz_Stop(void)
{
    return HAL_TIM_Base_Stop_IT(&htim11);
}

uint32_t BSP_Timer1kHz_GetTick(void)
{
    return bsp_timer_1khz_tick;
}

uint32_t BSP_Timer100Hz_GetTick(void)
{
    return bsp_timer_100hz_tick;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        bsp_timer_1khz_tick++;

        if (bsp_timer_1khz_callback != NULL)
        {
            bsp_timer_1khz_callback();
        }

        return;
    }

    if (htim->Instance == TIM11)
    {
        bsp_timer_100hz_tick++;

        if (bsp_timer_100hz_callback != NULL)
        {
            bsp_timer_100hz_callback();
        }
    }
}
