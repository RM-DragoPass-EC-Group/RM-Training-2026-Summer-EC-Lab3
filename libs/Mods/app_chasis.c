#include "app_chasis.h"

#include "EzTuner.h"
#include "drv/drv_motor.h"
#include "modules/module_pid.h"

#include <stddef.h>

#define APP_CHASIS_PI 3.14159265359f
#define APP_CHASIS_TWO_PI 6.28318530718f

static PID_t app_chasis_turn_pid;
static PID_t app_chasis_speed_pid[MOTOR_NUM];
static uint8_t app_chasis_turn_pid_inited = 0u;
static uint8_t app_chasis_speed_pid_inited = 0u;
static uint8_t app_chasis_motor_inited = 0u;

volatile AppChasisControlState_t app_chasis_control_state;

static float App_ChasisWrapAngleRad(float angle_rad)
{
    while (angle_rad > APP_CHASIS_PI)
    {
        angle_rad -= APP_CHASIS_TWO_PI;
    }

    while (angle_rad < -APP_CHASIS_PI)
    {
        angle_rad += APP_CHASIS_TWO_PI;
    }

    return angle_rad;
}

static void App_ChasisTurnPIDInit(void)
{
    PID_Init(&app_chasis_turn_pid,
             eztuner_chassis_turn_kp,
             eztuner_chassis_turn_ki,
             eztuner_chassis_turn_kd);
    PID_SetOutputLimit(&app_chasis_turn_pid,
                       -eztuner_chassis_turn_max_wz_radps,
                       eztuner_chassis_turn_max_wz_radps);
    PID_SetIntegralLimit(&app_chasis_turn_pid,
                         -eztuner_chassis_turn_integral_limit,
                         eztuner_chassis_turn_integral_limit);
    app_chasis_turn_pid_inited = 1u;
}

static void App_ChasisSpeedPIDInit(void)
{
    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_Init(&app_chasis_speed_pid[i],
                 eztuner_chassis_speed_kp,
                 eztuner_chassis_speed_ki,
                 eztuner_chassis_speed_kd);
        PID_SetOutputLimit(&app_chasis_speed_pid[i],
                           -eztuner_chassis_speed_max_output_percent,
                           eztuner_chassis_speed_max_output_percent);
        PID_SetIntegralLimit(&app_chasis_speed_pid[i],
                             -eztuner_chassis_speed_integral_limit,
                             eztuner_chassis_speed_integral_limit);
    }

    app_chasis_speed_pid_inited = 1u;
}

static void App_ChasisApplyTurnPIDTuning(void)
{
    PID_SetParam(&app_chasis_turn_pid,
                 eztuner_chassis_turn_kp,
                 eztuner_chassis_turn_ki,
                 eztuner_chassis_turn_kd);
    PID_SetOutputLimit(&app_chasis_turn_pid,
                       -eztuner_chassis_turn_max_wz_radps,
                       eztuner_chassis_turn_max_wz_radps);
    PID_SetIntegralLimit(&app_chasis_turn_pid,
                         -eztuner_chassis_turn_integral_limit,
                         eztuner_chassis_turn_integral_limit);
}

static void App_ChasisApplySpeedPIDTuning(void)
{
    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_SetParam(&app_chasis_speed_pid[i],
                     eztuner_chassis_speed_kp,
                     eztuner_chassis_speed_ki,
                     eztuner_chassis_speed_kd);
        PID_SetOutputLimit(&app_chasis_speed_pid[i],
                           -eztuner_chassis_speed_max_output_percent,
                           eztuner_chassis_speed_max_output_percent);
        PID_SetIntegralLimit(&app_chasis_speed_pid[i],
                             -eztuner_chassis_speed_integral_limit,
                             eztuner_chassis_speed_integral_limit);
    }
}

static void App_ChasisPIDInit(void)
{
    App_ChasisTurnPIDInit();
    App_ChasisSpeedPIDInit();
}

static void App_ChasisEnsurePIDInited(void)
{
    if (app_chasis_turn_pid_inited == 0u)
    {
        App_ChasisTurnPIDInit();
    }

    if (app_chasis_speed_pid_inited == 0u)
    {
        App_ChasisSpeedPIDInit();
    }
}

static void App_ChasisEnsureMotorInited(void)
{
    if (app_chasis_motor_inited == 0u)
    {
        Motor_Init();
        Motor_StopAll();
        app_chasis_motor_inited = 1u;
    }
}

static float App_ChasisGetWheelRPM(ChassisMecanumWheelRPM_t wheel_rpm, MotorId_t motor)
{
    switch (motor)
    {
        case MOTOR_FL:
            return wheel_rpm.fl_rpm;

        case MOTOR_FR:
            return wheel_rpm.fr_rpm;

        case MOTOR_RL:
            return wheel_rpm.rl_rpm;

        case MOTOR_RR:
            return wheel_rpm.rr_rpm;

        default:
            return 0.0f;
    }
}

static void App_ChasisSetWheelOutput(AppChasisWheelOutput_t *output_percent, MotorId_t motor, float value)
{
    if (output_percent == NULL)
    {
        return;
    }

    switch (motor)
    {
        case MOTOR_FL:
            output_percent->fl_percent = value;
            break;

        case MOTOR_FR:
            output_percent->fr_percent = value;
            break;

        case MOTOR_RL:
            output_percent->rl_percent = value;
            break;

        case MOTOR_RR:
            output_percent->rr_percent = value;
            break;

        default:
            break;
    }
}

static ChassisMecanumWheelRPM_t App_ChasisReadFeedbackRPM(void)
{
    ChassisMecanumWheelRPM_t feedback_rpm = {
        Motor_GetSpeedRPM(MOTOR_FL),
        Motor_GetSpeedRPM(MOTOR_FR),
        Motor_GetSpeedRPM(MOTOR_RL),
        Motor_GetSpeedRPM(MOTOR_RR),
    };

    return feedback_rpm;
}

static void App_ChasisClearControlState(void)
{
    app_chasis_control_state.cmd.vx_mps = 0.0f;
    app_chasis_control_state.cmd.vy_mps = 0.0f;
    app_chasis_control_state.cmd.wz_radps = 0.0f;

    app_chasis_control_state.target_rpm.fl_rpm = 0.0f;
    app_chasis_control_state.target_rpm.fr_rpm = 0.0f;
    app_chasis_control_state.target_rpm.rl_rpm = 0.0f;
    app_chasis_control_state.target_rpm.rr_rpm = 0.0f;

    app_chasis_control_state.feedback_rpm.fl_rpm = 0.0f;
    app_chasis_control_state.feedback_rpm.fr_rpm = 0.0f;
    app_chasis_control_state.feedback_rpm.rl_rpm = 0.0f;
    app_chasis_control_state.feedback_rpm.rr_rpm = 0.0f;

    app_chasis_control_state.output_percent.fl_percent = 0.0f;
    app_chasis_control_state.output_percent.fr_percent = 0.0f;
    app_chasis_control_state.output_percent.rl_percent = 0.0f;
    app_chasis_control_state.output_percent.rr_percent = 0.0f;

    app_chasis_control_state.update_cnt = 0u;
}

static void App_ChasisUpdateControlState(ChassisMecanumCmd_t cmd,
                                         ChassisMecanumWheelRPM_t target_rpm,
                                         ChassisMecanumWheelRPM_t feedback_rpm,
                                         AppChasisWheelOutput_t output_percent)
{
    app_chasis_control_state.cmd.vx_mps = cmd.vx_mps;
    app_chasis_control_state.cmd.vy_mps = cmd.vy_mps;
    app_chasis_control_state.cmd.wz_radps = cmd.wz_radps;

    app_chasis_control_state.target_rpm.fl_rpm = target_rpm.fl_rpm;
    app_chasis_control_state.target_rpm.fr_rpm = target_rpm.fr_rpm;
    app_chasis_control_state.target_rpm.rl_rpm = target_rpm.rl_rpm;
    app_chasis_control_state.target_rpm.rr_rpm = target_rpm.rr_rpm;

    app_chasis_control_state.feedback_rpm.fl_rpm = feedback_rpm.fl_rpm;
    app_chasis_control_state.feedback_rpm.fr_rpm = feedback_rpm.fr_rpm;
    app_chasis_control_state.feedback_rpm.rl_rpm = feedback_rpm.rl_rpm;
    app_chasis_control_state.feedback_rpm.rr_rpm = feedback_rpm.rr_rpm;

    app_chasis_control_state.output_percent.fl_percent = output_percent.fl_percent;
    app_chasis_control_state.output_percent.fr_percent = output_percent.fr_percent;
    app_chasis_control_state.output_percent.rl_percent = output_percent.rl_percent;
    app_chasis_control_state.output_percent.rr_percent = output_percent.rr_percent;

    app_chasis_control_state.update_cnt++;
}

static AppChasisWheelOutput_t App_ChasisRunWheelSpeedPID(ChassisMecanumWheelRPM_t target_rpm,
                                                         ChassisMecanumWheelRPM_t feedback_rpm,
                                                         float dt_s)
{
    AppChasisWheelOutput_t output_percent = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        MotorId_t motor = (MotorId_t)i;
        float target = App_ChasisGetWheelRPM(target_rpm, motor);
        float feedback = App_ChasisGetWheelRPM(feedback_rpm, motor);
        float output = PID_Calc(&app_chasis_speed_pid[i], target, feedback, dt_s);

        Motor_SetOutputPercent(motor, output);
        App_ChasisSetWheelOutput(&output_percent, motor, output);
    }

    return output_percent;
}

void App_ChasisInit(void)
{
    App_ChasisEnsureMotorInited();
    App_ChasisPIDInit();
    App_ChasisApplyTurnPIDTuning();
    App_ChasisApplySpeedPIDTuning();
    App_ChasisClearControlState();
}

void App_ChasisTurnPIDReset(void)
{
    if (app_chasis_turn_pid_inited == 0u)
    {
        App_ChasisTurnPIDInit();
        return;
    }

    PID_Reset(&app_chasis_turn_pid);
}

void App_ChasisTurnPIDSetParam(float kp, float ki, float kd)
{
    if (app_chasis_turn_pid_inited == 0u)
    {
        App_ChasisTurnPIDInit();
    }

    eztuner_chassis_turn_kp = kp;
    eztuner_chassis_turn_ki = ki;
    eztuner_chassis_turn_kd = kd;
    PID_SetParam(&app_chasis_turn_pid, kp, ki, kd);
}

void App_ChasisSpeedPIDReset(void)
{
    if (app_chasis_speed_pid_inited == 0u)
    {
        App_ChasisSpeedPIDInit();
        return;
    }

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_Reset(&app_chasis_speed_pid[i]);
    }
}

void App_ChasisSpeedPIDSetParam(float kp, float ki, float kd)
{
    if (app_chasis_speed_pid_inited == 0u)
    {
        App_ChasisSpeedPIDInit();
    }

    eztuner_chassis_speed_kp = kp;
    eztuner_chassis_speed_ki = ki;
    eztuner_chassis_speed_kd = kd;

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_SetParam(&app_chasis_speed_pid[i], kp, ki, kd);
    }
}

float App_ChasisTurnPIDCalc(float target_yaw_rad, float gyro_yaw_rad, float dt_s)
{
    float yaw_error_rad;

    if (app_chasis_turn_pid_inited == 0u)
    {
        App_ChasisTurnPIDInit();
    }

    yaw_error_rad = App_ChasisWrapAngleRad(target_yaw_rad - gyro_yaw_rad);
    App_ChasisApplyTurnPIDTuning();

    return PID_Calc(&app_chasis_turn_pid, yaw_error_rad, 0.0f, dt_s);
}

ChassisMecanumWheelRPM_t App_ChasisHeadingSolve(AppChasisHeadingCmd_t cmd, float dt_s)
{
    ChassisMecanumCmd_t mecanum_cmd = {
        cmd.vx_mps,
        cmd.vy_mps,
        App_ChasisTurnPIDCalc(cmd.target_yaw_rad, cmd.gyro_yaw_rad, dt_s),
    };
    ChassisMecanumWheelRPM_t wheel_rpm = ChassisMecanum_InverseKinematics(mecanum_cmd);

    ChassisMecanum_LimitWheelRPM(&wheel_rpm, CHASSIS_MECANUM_MAX_WHEEL_RPM);

    return wheel_rpm;
}

void App_ChasisVelocityControl(ChassisMecanumCmd_t cmd, uint32_t dt_ms)
{
    ChassisMecanumWheelRPM_t target_rpm;
    ChassisMecanumWheelRPM_t feedback_rpm;
    AppChasisWheelOutput_t output_percent;
    float dt_s;

    if (dt_ms == 0u)
    {
        return;
    }

    App_ChasisEnsureMotorInited();
    App_ChasisEnsurePIDInited();
    App_ChasisApplySpeedPIDTuning();

    dt_s = (float)dt_ms / 1000.0f;
    Motor_UpdateSpeed(dt_ms);

    target_rpm = ChassisMecanum_InverseKinematics(cmd);
    ChassisMecanum_LimitWheelRPM(&target_rpm, CHASSIS_MECANUM_MAX_WHEEL_RPM);
    feedback_rpm = App_ChasisReadFeedbackRPM();
    output_percent = App_ChasisRunWheelSpeedPID(target_rpm, feedback_rpm, dt_s);

    App_ChasisUpdateControlState(cmd, target_rpm, feedback_rpm, output_percent);
}

void App_ChasisHeadingControl(AppChasisHeadingCmd_t cmd, uint32_t dt_ms)
{
    ChassisMecanumCmd_t mecanum_cmd;
    float dt_s;

    if (dt_ms == 0u)
    {
        return;
    }

    App_ChasisEnsurePIDInited();

    dt_s = (float)dt_ms / 1000.0f;
    mecanum_cmd.vx_mps = cmd.vx_mps;
    mecanum_cmd.vy_mps = cmd.vy_mps;
    mecanum_cmd.wz_radps = App_ChasisTurnPIDCalc(cmd.target_yaw_rad, cmd.gyro_yaw_rad, dt_s);

    App_ChasisVelocityControl(mecanum_cmd, dt_ms);
}

void App_ChasisStop(void)
{
    App_ChasisEnsureMotorInited();
    Motor_StopAll();
    App_ChasisSpeedPIDReset();
    App_ChasisClearControlState();
}

void App_ChasisBrake(void)
{
    App_ChasisEnsureMotorInited();

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        Motor_Brake((MotorId_t)i);
    }

    App_ChasisSpeedPIDReset();
    App_ChasisClearControlState();
}

AppChasisControlState_t App_ChasisGetControlState(void)
{
    AppChasisControlState_t state;

    state.cmd.vx_mps = app_chasis_control_state.cmd.vx_mps;
    state.cmd.vy_mps = app_chasis_control_state.cmd.vy_mps;
    state.cmd.wz_radps = app_chasis_control_state.cmd.wz_radps;

    state.target_rpm.fl_rpm = app_chasis_control_state.target_rpm.fl_rpm;
    state.target_rpm.fr_rpm = app_chasis_control_state.target_rpm.fr_rpm;
    state.target_rpm.rl_rpm = app_chasis_control_state.target_rpm.rl_rpm;
    state.target_rpm.rr_rpm = app_chasis_control_state.target_rpm.rr_rpm;

    state.feedback_rpm.fl_rpm = app_chasis_control_state.feedback_rpm.fl_rpm;
    state.feedback_rpm.fr_rpm = app_chasis_control_state.feedback_rpm.fr_rpm;
    state.feedback_rpm.rl_rpm = app_chasis_control_state.feedback_rpm.rl_rpm;
    state.feedback_rpm.rr_rpm = app_chasis_control_state.feedback_rpm.rr_rpm;

    state.output_percent.fl_percent = app_chasis_control_state.output_percent.fl_percent;
    state.output_percent.fr_percent = app_chasis_control_state.output_percent.fr_percent;
    state.output_percent.rl_percent = app_chasis_control_state.output_percent.rl_percent;
    state.output_percent.rr_percent = app_chasis_control_state.output_percent.rr_percent;

    state.update_cnt = app_chasis_control_state.update_cnt;

    return state;
}
