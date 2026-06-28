/**
 * @file drv_motor.h
 * @brief Motor driver speed and output interface.
 *
 * This driver maps motors to low-level BSP encoder channels and converts
 * encoder counts to output-shaft rpm, and controls motor output through the
 * TB6612 BSP layer.
 */

#ifndef __DRV_MOTOR_H__
#define __DRV_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "app/EzTuner.h"

/**
 * @brief Quadrature decode multiplier used by the timer encoder configuration.
 *
 * Use 4u for x4 decode. If the TIM encoder mode only counts one edge or one
 * channel, set this value to match the actual hardware count multiplier.
 */
#define MOTOR_ENCODER_QUADRATURE_MULTIPLE 4u

/**
 * @brief Motor-side encoder line count.
 *
 * Replace these default integer values with the real encoder resolution before
 * using the rpm interface.
 */
#define MOTOR_FL_ENCODER_RESOLUTION 11u
#define MOTOR_FR_ENCODER_RESOLUTION 11u
#define MOTOR_RL_ENCODER_RESOLUTION 11u
#define MOTOR_RR_ENCODER_RESOLUTION 11u

/**
 * @brief Motor reduction ratio from motor shaft to output shaft.
 *
 * For example, use 19.2f when the motor rotates 19.2 turns for one output-shaft
 * turn. If the encoder is already mounted on the output shaft, use 1.0f.
 */
#define MOTOR_FL_REDUCTION_RATIO 21.3f
#define MOTOR_FR_REDUCTION_RATIO 21.3f
#define MOTOR_RL_REDUCTION_RATIO 21.3f
#define MOTOR_RR_REDUCTION_RATIO 21.3f

#define MOTOR_FL_OUTPUT_REVERSE EZTUNER_MOTOR_FL_OUTPUT_REVERSE
#define MOTOR_FR_OUTPUT_REVERSE EZTUNER_MOTOR_FR_OUTPUT_REVERSE
#define MOTOR_RL_OUTPUT_REVERSE EZTUNER_MOTOR_RL_OUTPUT_REVERSE
#define MOTOR_RR_OUTPUT_REVERSE EZTUNER_MOTOR_RR_OUTPUT_REVERSE

#define MOTOR_FL_ENCODER_REVERSE EZTUNER_MOTOR_FL_ENCODER_REVERSE
#define MOTOR_FR_ENCODER_REVERSE EZTUNER_MOTOR_FR_ENCODER_REVERSE
#define MOTOR_RL_ENCODER_REVERSE EZTUNER_MOTOR_RL_ENCODER_REVERSE
#define MOTOR_RR_ENCODER_REVERSE EZTUNER_MOTOR_RR_ENCODER_REVERSE



/**
 * @brief Convert motor-side encoder resolution and reduction ratio to counts per output revolution.
 *
 * @param encoder_resolution Motor-side encoder line count.
 * @param reduction_ratio Motor-to-output reduction ratio.
 * @return Encoder counts per output-shaft revolution.
 */
#define MOTOR_ENCODER_CNT_PER_OUTPUT_REV(encoder_resolution, reduction_ratio) \
    ((float)(encoder_resolution) * MOTOR_ENCODER_QUADRATURE_MULTIPLE * (reduction_ratio))

/**
 * @brief Motor index used by motor driver APIs.
 */
typedef enum
{
    /** Front-left motor. */
    MOTOR_FL = 0,
    /** Front-right motor. */
    MOTOR_FR,
    /** Rear-left motor. */
    MOTOR_RL,
    /** Rear-right motor. */
    MOTOR_RR,
    /** Number of motors. */
    MOTOR_NUM
} MotorId_t;

/**
 * @brief Motor output mode.
 */
typedef enum
{
    /** IN1=0, IN2=0, PWM=0. */
    MOTOR_OUTPUT_COAST = 0,
    /** IN1=1, IN2=0. */
    MOTOR_OUTPUT_FORWARD,
    /** IN1=0, IN2=1. */
    MOTOR_OUTPUT_REVERSE,
    /** IN1=1, IN2=1, PWM=max. */
    MOTOR_OUTPUT_BRAKE
} MotorOutputMode_t;

/**
 * @brief Cached motor speed state.
 */
typedef struct
{
    /** Latest speed in encoder counts per second. */
    int32_t speed_cnt_per_s;
    /** Latest output-shaft speed in rpm. */
    float speed_rpm;
    /** Encoder count difference during the latest update period. */
    int32_t delta_cnt;
    /** Raw timer counter value sampled during the latest update. */
    uint32_t counter;
} MotorSpeed_t;

/**
 * @brief Start motor BSP resources and reset motor speed cache.
 *
 * Call this after MX_TIM1_Init(), MX_TIM2_Init(), MX_TIM3_Init(), and
 * MX_TIM4_Init(). This also starts the motor PWM outputs.
 */
void Motor_Init(void);

/**
 * @brief Set motor output mode and raw PWM duty.
 *
 * @param motor Motor index.
 * @param mode Output mode.
 * @param duty PWM compare value. It is clamped to the TIM auto-reload value.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Motor_SetOutput(MotorId_t motor, MotorOutputMode_t mode, uint16_t duty);

/**
 * @brief Set motor output by signed percentage.
 *
 * Positive percent drives forward, negative percent drives reverse, and zero
 * coasts. The value is clamped to [-100, 100].
 *
 * @param motor Motor index.
 * @param speed_percent Signed output percentage.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Motor_SetOutputPercent(MotorId_t motor, float speed_percent);

/**
 * @brief Coast-stop one motor.
 *
 * @param motor Motor index.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Motor_Stop(MotorId_t motor);

/**
 * @brief Short-brake one motor.
 *
 * @param motor Motor index.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Motor_Brake(MotorId_t motor);

/**
 * @brief Coast-stop all motors.
 *
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Motor_StopAll(void);

/**
 * @brief Update all motor speed values.
 *
 * This function refreshes the BSP encoder counters, then converts each motor
 * speed to output-shaft rpm.
 *
 * @param dt_ms Time since the previous update, in milliseconds.
 */
void Motor_UpdateSpeed(uint32_t dt_ms);

/**
 * @brief Get the latest motor speed in output-shaft rpm.
 *
 * @param motor Motor index.
 * @return Latest output-shaft speed in rpm. Returns 0.0f for invalid index.
 */
float Motor_GetSpeedRPM(MotorId_t motor);

/**
 * @brief Get the latest motor speed in encoder counts per second.
 *
 * @param motor Motor index.
 * @return Latest encoder speed in counts per second. Returns 0 for invalid index.
 */
int32_t Motor_GetSpeedCntPerS(MotorId_t motor);

/**
 * @brief Get the complete cached motor speed state.
 *
 * @param motor Motor index.
 * @return Cached motor speed state. Returns zero-filled state for invalid index.
 */
MotorSpeed_t Motor_GetSpeedState(MotorId_t motor);

#ifdef __cplusplus
}
#endif

#endif
