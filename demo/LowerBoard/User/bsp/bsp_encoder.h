/**
 * @file bsp_encoder.h
 * @brief Low-level PWM timer and encoder counter interface.
 *
 * This BSP module owns TIM encoder startup, counter sampling, and counter
 * wraparound handling. Motor-specific concepts such as wheel position,
 * encoder resolution, reduction ratio, and rpm conversion are implemented in
 * the motor driver layer.
 */

#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Low-level encoder channel index.
 */
typedef enum
{
    /** Encoder channel driven by TIM1. */
    PWM_ENCODER_CH1 = 0,
    /** Encoder channel driven by TIM2. */
    PWM_ENCODER_CH2,
    /** Encoder channel driven by TIM3. */
    PWM_ENCODER_CH3,
    /** Encoder channel driven by TIM4. */
    PWM_ENCODER_CH4,
    /** Number of encoder channels. */
    PWM_ENCODER_NUM
} PWM_EncoderId_t;

/**
 * @brief Cached low-level encoder state.
 */
typedef struct
{
    /** Latest speed in encoder timer counts per second. */
    int32_t speed_cnt_per_s;
    /** Encoder count difference during the latest update period. */
    int32_t delta_cnt;
    /** Raw timer counter value sampled during the latest update. */
    uint32_t counter;
} PWM_EncoderState_t;

/**
 * @brief Start all encoder timers and reset cached low-level state.
 *
 * Call this after MX_TIM1_Init(), MX_TIM2_Init(), MX_TIM3_Init(), and
 * MX_TIM4_Init().
 */
void PWM_EncoderInit(void);

/**
 * @brief Update all low-level encoder speeds.
 *
 * This function handles 16-bit and 32-bit timer counter wraparound.
 *
 * @param dt_ms Time since the previous update, in milliseconds.
 */
void PWM_EncoderUpdate(uint32_t dt_ms);

/**
 * @brief Get the latest speed in encoder timer counts per second.
 *
 * @param encoder Encoder channel index.
 * @return Latest encoder speed in counts per second. Returns 0 for invalid index.
 */
int32_t PWM_GetEncoderSpeedCntPerS(PWM_EncoderId_t encoder);

/**
 * @brief Get the latest encoder count delta.
 *
 * @param encoder Encoder channel index.
 * @return Count difference from the latest update period. Returns 0 for invalid index.
 */
int32_t PWM_GetEncoderDelta(PWM_EncoderId_t encoder);

/**
 * @brief Get the complete cached low-level encoder state.
 *
 * @param encoder Encoder channel index.
 * @return Cached encoder state. Returns zero-filled state for invalid index.
 */
PWM_EncoderState_t PWM_GetEncoderState(PWM_EncoderId_t encoder);

#ifdef __cplusplus
}
#endif

#endif
