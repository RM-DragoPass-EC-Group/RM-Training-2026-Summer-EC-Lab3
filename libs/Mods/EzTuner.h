/**
 * @file EzTuner.h
 * @brief Central tuning parameters for chassis PID loops.
 */

#ifndef __EZTUNER_H__
#define __EZTUNER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Heading PID default parameters.
 *
 * Input is yaw error in radians, output is yaw speed in rad/s.
 */
#define EZTUNER_CHASSIS_TURN_KP 0.0f
#define EZTUNER_CHASSIS_TURN_KI 0.0f
#define EZTUNER_CHASSIS_TURN_KD 0.0f

/**
 * @brief Heading-loop default output and integral limits.
 */
#define EZTUNER_CHASSIS_TURN_MAX_WZ_RADPS 3.14f
#define EZTUNER_CHASSIS_TURN_INTEGRAL_LIMIT 1.0f

/**
 * @brief Wheel speed PID default parameters.
 *
 * Input and feedback are wheel output-shaft rpm. Output is signed motor output
 * percentage for Motor_SetOutputPercent().
 */
#define EZTUNER_CHASSIS_SPEED_KP 0.8f
#define EZTUNER_CHASSIS_SPEED_KI 0.1f
#define EZTUNER_CHASSIS_SPEED_KD 0.0f

/**
 * @brief Wheel speed-loop default output and integral limits.
 */
#define EZTUNER_CHASSIS_SPEED_MAX_OUTPUT_PERCENT 100.0f
#define EZTUNER_CHASSIS_SPEED_INTEGRAL_LIMIT 1000.0f

/**
 * @brief Motor direction tuning switches.
 *
 * Set output reverse to 1 when positive motor command makes the wheel rotate in
 * the negative chassis direction. Set encoder reverse to 1 when positive wheel
 * rotation is measured as negative rpm.
 */
#define EZTUNER_MOTOR_FL_OUTPUT_REVERSE 0u
#define EZTUNER_MOTOR_FR_OUTPUT_REVERSE 0u
#define EZTUNER_MOTOR_RL_OUTPUT_REVERSE 1u
#define EZTUNER_MOTOR_RR_OUTPUT_REVERSE 1u

#define EZTUNER_MOTOR_FL_ENCODER_REVERSE 1u
#define EZTUNER_MOTOR_FR_ENCODER_REVERSE 1u
#define EZTUNER_MOTOR_RL_ENCODER_REVERSE 1u
#define EZTUNER_MOTOR_RR_ENCODER_REVERSE 0u








/**
 * @brief Runtime heading PID gains watched/modified by EzTuner or debugger.
 */
extern volatile float eztuner_chassis_turn_kp;
extern volatile float eztuner_chassis_turn_ki;
extern volatile float eztuner_chassis_turn_kd;

/**
 * @brief Runtime heading PID limits watched/modified by EzTuner or debugger.
 */
extern volatile float eztuner_chassis_turn_max_wz_radps;
extern volatile float eztuner_chassis_turn_integral_limit;

/**
 * @brief Runtime wheel speed PID gains watched/modified by EzTuner or debugger.
 */
extern volatile float eztuner_chassis_speed_kp;
extern volatile float eztuner_chassis_speed_ki;
extern volatile float eztuner_chassis_speed_kd;

/**
 * @brief Runtime wheel speed PID limits watched/modified by EzTuner or debugger.
 */
extern volatile float eztuner_chassis_speed_max_output_percent;
extern volatile float eztuner_chassis_speed_integral_limit;

#ifdef __cplusplus
}
#endif

#endif
