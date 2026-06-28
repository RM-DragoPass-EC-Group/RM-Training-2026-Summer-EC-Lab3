/**
 * @file bsp_led.h
 * @brief Low-level user LED interface.
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);
void BSP_LED_Set(GPIO_PinState state);

#ifdef __cplusplus
}
#endif

#endif
