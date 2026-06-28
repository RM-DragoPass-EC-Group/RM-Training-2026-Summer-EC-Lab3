/**
 * @file module_chassis_mecanum.h
 * @brief X-layout mecanum chassis kinematics.
 */

#ifndef __MODULE_CHASSIS_MECANUM_H__
#define __MODULE_CHASSIS_MECANUM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Wheel radius in meters.
 */
#define CHASSIS_MECANUM_WHEEL_RADIUS_M 0.0325f

/**
 * @brief Distance from chassis center to front/rear wheel contact point in meters.
 */
#define CHASSIS_MECANUM_HALF_LENGTH_M 0.1f

/**
 * @brief Distance from chassis center to left/right wheel contact point in meters.
 */
#define CHASSIS_MECANUM_HALF_WIDTH_M 0.1225f

/**
 * @brief Mechanical speed limit for one wheel in output-shaft rpm.
 *
 * Set this to your motor output limit. It is used only by the limit helper.
 */
#define CHASSIS_MECANUM_MAX_WHEEL_RPM 282.0f

/**
 * @brief Chassis velocity command.
 *
 * Coordinate convention: +vx is forward, +vy is left, +wz is counterclockwise.
 */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} ChassisMecanumCmd_t;

/**
 * @brief Four wheel target speeds.
 */
typedef struct
{
    float fl_rpm;
    float fr_rpm;
    float rl_rpm;
    float rr_rpm;
} ChassisMecanumWheelRPM_t;

/**
 * @brief Solve X-layout mecanum inverse kinematics.
 *
 * @param cmd Chassis velocity command.
 * @return Four wheel output-shaft target speeds in rpm.
 */
ChassisMecanumWheelRPM_t ChassisMecanum_InverseKinematics(ChassisMecanumCmd_t cmd);

/**
 * @brief Solve X-layout mecanum forward kinematics from wheel rpm.
 *
 * @param wheel_rpm Four wheel output-shaft speeds in rpm.
 * @return Chassis velocity calculated from wheel speeds.
 */
ChassisMecanumCmd_t ChassisMecanum_ForwardKinematics(ChassisMecanumWheelRPM_t wheel_rpm);

/**
 * @brief Scale wheel speeds proportionally so no wheel exceeds max_abs_rpm.
 *
 * @param wheel_rpm Wheel speed targets to limit in-place.
 * @param max_abs_rpm Maximum absolute wheel rpm.
 */
void ChassisMecanum_LimitWheelRPM(ChassisMecanumWheelRPM_t *wheel_rpm, float max_abs_rpm);

#ifdef __cplusplus
}
#endif

#endif
