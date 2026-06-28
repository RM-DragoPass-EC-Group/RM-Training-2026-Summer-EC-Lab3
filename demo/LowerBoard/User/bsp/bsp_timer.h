/**
 * @file bsp_timer.h
 * @brief BSP timer interrupt interface based on TIM10 and TIM11.
 *
 * This module starts CubeMX-configured timer interrupts and lets user code
 * register callbacks that run from timer interrupt context.
 */

#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief User callback type for the 1 kHz TIM10 interrupt.
 *
 * Keep the callback short. It is called inside interrupt context.
 */
typedef void (*BSP_Timer1kHzCallback_t)(void);

/**
 * @brief User callback type for the 100 Hz TIM11 interrupt.
 *
 * Keep the callback short. It is called inside interrupt context.
 */
typedef void (*BSP_Timer100HzCallback_t)(void);

/**
 * @brief Start the CubeMX-configured TIM10 1 kHz interrupt timer.
 *
 * Call this after MX_TIM10_Init(). The timer frequency is controlled by the
 * CubeMX-generated TIM10 prescaler and period settings.
 *
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_Timer1kHz_Init(void);

/**
 * @brief Start the CubeMX-configured TIM11 100 Hz interrupt timer.
 *
 * Call this after MX_TIM11_Init(). The timer frequency is controlled by the
 * CubeMX-generated TIM11 prescaler and period settings.
 *
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_Timer100Hz_Init(void);

/**
 * @brief Register the user callback called every 1 ms.
 *
 * Passing NULL unregisters the current callback.
 *
 * @param callback User callback function.
 */
void BSP_Timer1kHz_RegisterCallback(BSP_Timer1kHzCallback_t callback);

/**
 * @brief Register the user callback called every 10 ms.
 *
 * Passing NULL unregisters the current callback.
 *
 * @param callback User callback function.
 */
void BSP_Timer100Hz_RegisterCallback(BSP_Timer100HzCallback_t callback);

/**
 * @brief Stop the TIM10 1 kHz interrupt.
 *
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_Timer1kHz_Stop(void);

/**
 * @brief Stop the TIM11 100 Hz interrupt.
 *
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_Timer100Hz_Stop(void);

/**
 * @brief Get the internal 1 ms tick count maintained by this module.
 *
 * @return Tick count incremented once per TIM10 interrupt.
 */
uint32_t BSP_Timer1kHz_GetTick(void);

/**
 * @brief Get the internal 10 ms tick count maintained by this module.
 *
 * @return Tick count incremented once per TIM11 interrupt.
 */
uint32_t BSP_Timer100Hz_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif
