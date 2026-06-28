/**
 * @file app_under_board_task.h
 * @brief Main under-board application task scheduler.
 */

#ifndef __APP_UNDER_BOARD_TASK_H__
#define __APP_UNDER_BOARD_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Control task period driven by TIM11 100 Hz callback.
 */
#define APP_UNDER_BOARD_TASK_PERIOD_MS 10u

/**
 * @brief Upper-computer command timeout.
 *
 * If no valid UART packet is received for this time, chassis output is disabled.
 */
#define APP_UNDER_BOARD_UART_TIMEOUT_MS 500u

/**
 * @brief LED toggle interval.
 */
#define APP_UNDER_BOARD_LED_TOGGLE_MS 500u

/**
 * @brief Set by 100 Hz timer callback and cleared by App_UnderBoardTaskRun().
 */
extern volatile uint8_t app_under_board_task_100hz_flag;

/**
 * @brief Number of executed 100 Hz task steps.
 */
extern volatile uint32_t app_under_board_task_tick;

/**
 * @brief 1 when valid upper-computer command is not timed out.
 */
extern volatile uint8_t app_under_board_task_online;

/**
 * @brief Number of command timeout events.
 */
extern volatile uint32_t app_under_board_task_timeout_count;

/**
 * @brief Initialize under-board task resources.
 *
 * Call this after CubeMX peripheral initialization.
 */
void App_UnderBoardTaskInit(void);

/**
 * @brief Run under-board business logic from the main while loop.
 *
 * This function returns immediately when the 100 Hz flag is not set.
 */
void App_UnderBoardTaskRun(void);

#ifdef __cplusplus
}
#endif

#endif
