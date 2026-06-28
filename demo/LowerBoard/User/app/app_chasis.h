/**
 * @file app_chasis.h
 * @brief App-level chassis control helpers.
 *
 * This layer owns heading control using gyro yaw from the upper computer. The
 * mecanum module only provides pure kinematics.
 */

#ifndef __APP_CHASIS_H__
#define __APP_CHASIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "EzTuner.h"
#include "modules/module_chassis_mecanum.h"

/**
 * @brief Chassis PID parameters are defined in EzTuner.h.
 */
#define APP_CHASIS_TURN_KP EZTUNER_CHASSIS_TURN_KP
#define APP_CHASIS_TURN_KI EZTUNER_CHASSIS_TURN_KI
#define APP_CHASIS_TURN_KD EZTUNER_CHASSIS_TURN_KD
#define APP_CHASIS_TURN_MAX_WZ_RADPS EZTUNER_CHASSIS_TURN_MAX_WZ_RADPS
#define APP_CHASIS_TURN_INTEGRAL_LIMIT EZTUNER_CHASSIS_TURN_INTEGRAL_LIMIT

#define APP_CHASIS_SPEED_KP EZTUNER_CHASSIS_SPEED_KP
#define APP_CHASIS_SPEED_KI EZTUNER_CHASSIS_SPEED_KI
#define APP_CHASIS_SPEED_KD EZTUNER_CHASSIS_SPEED_KD
#define APP_CHASIS_SPEED_MAX_OUTPUT_PERCENT EZTUNER_CHASSIS_SPEED_MAX_OUTPUT_PERCENT
#define APP_CHASIS_SPEED_INTEGRAL_LIMIT EZTUNER_CHASSIS_SPEED_INTEGRAL_LIMIT


/**
 * @brief Chassis command with heading control.
 *
 * Coordinate convention: +vx is forward, +vy is left. +yaw and +wz are
 * counterclockwise.
 */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float target_yaw_rad;
    float gyro_yaw_rad;
} AppChasisHeadingCmd_t;

/**
 * @brief Four signed motor output percentages.
 */
typedef struct
{
    float fl_percent;
    float fr_percent;
    float rl_percent;
    float rr_percent;
} AppChasisWheelOutput_t;

/**
 * @brief Cached chassis closed-loop control state for debugging.
 */
typedef struct
{
    ChassisMecanumCmd_t cmd;
    ChassisMecanumWheelRPM_t target_rpm;
    ChassisMecanumWheelRPM_t feedback_rpm;
    AppChasisWheelOutput_t output_percent;
    uint32_t update_cnt;
} AppChasisControlState_t;

/**
 * @brief Latest chassis closed-loop control state.
 *
 * This variable is intentionally global so it can be watched directly in the
 * debugger during tuning.
 */
extern volatile AppChasisControlState_t app_chasis_control_state;

/**
 * @brief Initialize chassis app layer.
 *
 * This initializes motor BSP/DRV resources, heading PID, and four wheel speed
 * PIDs. Call it after CubeMX peripheral init.
 */
void App_ChasisInit(void);

/**
 * @brief Reset heading PID runtime state.
 */
void App_ChasisTurnPIDReset(void);

/**
 * @brief Update heading PID parameters.
 *
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void App_ChasisTurnPIDSetParam(float kp, float ki, float kd);

/**
 * @brief Reset all wheel speed PID runtime states.
 */
void App_ChasisSpeedPIDReset(void);

/**
 * @brief Update all wheel speed PID parameters.
 *
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void App_ChasisSpeedPIDSetParam(float kp, float ki, float kd);

/**
 * @brief Calculate yaw speed command from target yaw and gyro yaw.
 *
 * The yaw error is wrapped into [-pi, pi] before PID calculation.
 *
 * @param target_yaw_rad Target yaw in radians.
 * @param gyro_yaw_rad Current gyro yaw in radians.
 * @param dt_s Control period in seconds.
 * @return Yaw speed command in rad/s.
 */
float App_ChasisTurnPIDCalc(float target_yaw_rad, float gyro_yaw_rad, float dt_s);

/**
 * @brief Solve wheel rpm target with heading control.
 *
 * @param cmd Chassis translation command and gyro yaw feedback.
 * @param dt_s Control period in seconds.
 * @return Four wheel output-shaft target speeds in rpm.
 */
ChassisMecanumWheelRPM_t App_ChasisHeadingSolve(AppChasisHeadingCmd_t cmd, float dt_s);

/**
 * @brief Run one closed-loop chassis velocity control step.
 *
 * This function updates encoder feedback, solves mecanum wheel target rpm,
 * runs four wheel speed PIDs, then writes signed output percentage to motors.
 *
 * @param cmd Chassis velocity command.
 * @param dt_ms Control period in milliseconds.
 */
void App_ChasisVelocityControl(ChassisMecanumCmd_t cmd, uint32_t dt_ms);

/**
 * @brief Run one closed-loop chassis velocity control step with heading hold.
 *
 * The heading PID generates wz from target_yaw_rad and gyro_yaw_rad, then the
 * wheel speed loop controls the four motors.
 *
 * @param cmd Chassis translation command and gyro yaw feedback.
 * @param dt_ms Control period in milliseconds.
 */
void App_ChasisHeadingControl(AppChasisHeadingCmd_t cmd, uint32_t dt_ms);

/**
 * @brief Coast-stop all chassis motors and reset wheel speed PIDs.
 */
void App_ChasisStop(void);

/**
 * @brief Short-brake all chassis motors and reset wheel speed PIDs.
 */
void App_ChasisBrake(void);

/**
 * @brief Get a copy of the latest chassis closed-loop control state.
 *
 * @return Latest cached control state.
 */
AppChasisControlState_t App_ChasisGetControlState(void);

#ifdef __cplusplus
}
#endif

#endif


