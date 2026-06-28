/**
 * @file app_test.h
 * @brief Temporary app-level test hooks.
 *
 * These APIs are meant for quick debugger tests. Include this header in
 * main.c and call the test function before entering while(1), then comment out
 * that single call when normal development resumes.
 */

#ifndef __APP_TEST_H__
#define __APP_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern volatile float app_test_motor_fl_rpm;
extern volatile float app_test_motor_fr_rpm;
extern volatile float app_test_motor_rl_rpm;
extern volatile float app_test_motor_rr_rpm;

extern volatile float app_test_motor_fl_mps;
extern volatile float app_test_motor_fr_mps;
extern volatile float app_test_motor_rl_mps;
extern volatile float app_test_motor_rr_mps;

extern volatile int32_t app_test_motor_fl_cnt_per_s;
extern volatile int32_t app_test_motor_fr_cnt_per_s;
extern volatile int32_t app_test_motor_rl_cnt_per_s;
extern volatile int32_t app_test_motor_rr_cnt_per_s;

extern volatile uint32_t app_test_motor_speed_update_cnt;

extern volatile float app_test_motor_fl_output_percent;
extern volatile float app_test_motor_fr_output_percent;
extern volatile float app_test_motor_rl_output_percent;
extern volatile float app_test_motor_rr_output_percent;

extern volatile float app_test_motor_fl_target_rpm;
extern volatile float app_test_motor_fr_target_rpm;
extern volatile float app_test_motor_rl_target_rpm;
extern volatile float app_test_motor_rr_target_rpm;

extern volatile float app_test_chassis_target_vx_mps;
extern volatile float app_test_chassis_target_vy_mps;
extern volatile float app_test_chassis_vx_mps;
extern volatile float app_test_chassis_vy_mps;
extern volatile float app_test_chassis_wz_radps;

extern volatile uint8_t app_test_motor_current;
extern volatile uint8_t app_test_motor_phase;
extern volatile uint16_t app_test_motor_phase_tick;
extern volatile uint8_t app_test_motor_sequence_done;

extern volatile uint8_t app_test_motor_pid_phase;
extern volatile uint16_t app_test_motor_pid_phase_tick;
extern volatile uint32_t app_test_motor_pid_cycle_cnt;

/**
 * @brief Copy current chassis-control target rpm and output into app_test globals.
 *
 * Call this from a temporary debug path when normal chassis control is running,
 * then watch app_test_motor_*_target_rpm and app_test_motor_*_output_percent
 * in the debugger.
 */
void App_TestUpdateChassisTargetRPMDebug(void);

/**
 * @brief Start the temporary 100 Hz motor sequence test.
 *
 * Call this after MX_TIM1/2/3/4_Init() and MX_TIM11_Init(), before entering
 * while(1). It drives FL, FR, RL, and RR one by one:
 * 0 -> +max in 2 s, +max -> 0 in 2 s, 0 -> -max in 2 s, -max -> 0 in 2 s,
 * then brake-locks that motor and advances to the next motor.
 */
void App_TestMotorSequenceStart(void);

/**
 * @brief Start the temporary 100 Hz four-wheel PID speed test.
 *
 * It ramps all wheels to +140 rpm over 5 s, holds for 30 s, ramps to 0 over
 * 5 s, then repeats the same profile at -140 rpm. The sequence loops forever.
 */
void App_TestMotorPIDStart(void);

/**
 * @brief Start the temporary 100 Hz four-wheel PID tuning test.
 *
 * It keeps all wheels driving forward at 70% of CHASSIS_MECANUM_MAX_WHEEL_RPM.
 * Wheel speed PID parameters are refreshed from EzTuner runtime variables every
 * control cycle so they can be adjusted while debugging.
 */
void App_TestMotorPID70PercentForwardStart(void);

/**
 * @brief Compatibility alias for App_TestMotorSequenceStart().
 */
void App_TestMotorSpeedStart(void);

#ifdef __cplusplus
}
#endif

#endif
