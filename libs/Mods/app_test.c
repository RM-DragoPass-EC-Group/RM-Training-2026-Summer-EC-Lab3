#include "app_test.h"

#include "app_chasis.h"
#include "bsp/bsp_timer.h"
#include "drv/drv_motor.h"
#include "EzTuner.h"
#include "modules/module_chassis_mecanum.h"
#include "modules/module_pid.h"

#define APP_TEST_MOTOR_SPEED_PERIOD_MS 10u
#define APP_TEST_MOTOR_CONTROL_PERIOD_S 0.01f
#define APP_TEST_MOTOR_RAMP_TIME_MS 2000u
#define APP_TEST_MOTOR_RAMP_TICKS (APP_TEST_MOTOR_RAMP_TIME_MS / APP_TEST_MOTOR_SPEED_PERIOD_MS)
#define APP_TEST_MOTOR_MAX_OUTPUT_PERCENT 100.0f

#define APP_TEST_PID_TARGET_RPM 140.0f
#define APP_TEST_PID_RAMP_TIME_MS 5000u
#define APP_TEST_PID_HOLD_TIME_MS 30000u
#define APP_TEST_PID_RAMP_TICKS (APP_TEST_PID_RAMP_TIME_MS / APP_TEST_MOTOR_SPEED_PERIOD_MS)
#define APP_TEST_PID_HOLD_TICKS (APP_TEST_PID_HOLD_TIME_MS / APP_TEST_MOTOR_SPEED_PERIOD_MS)
#define APP_TEST_PID_KP 0.4f
#define APP_TEST_PID_KI 0.05f
#define APP_TEST_PID_KD 0.0f
#define APP_TEST_PID_INTEGRAL_LIMIT 1000.0f
#define APP_TEST_PID_TUNE_FORWARD_PERCENT 70.0f
#define APP_TEST_PID_TUNE_TARGET_RPM \
    (CHASSIS_MECANUM_MAX_WHEEL_RPM * APP_TEST_PID_TUNE_FORWARD_PERCENT / 100.0f)
#define APP_TEST_RPM_TO_RADPS (6.28318530718f / 60.0f)

typedef enum
{
    APP_TEST_PHASE_POS_ACCEL = 0,
    APP_TEST_PHASE_POS_DECEL,
    APP_TEST_PHASE_NEG_ACCEL,
    APP_TEST_PHASE_NEG_DECEL,
    APP_TEST_PHASE_BRAKE,
    APP_TEST_PHASE_DONE
} App_TestMotorPhase_t;

typedef enum
{
    APP_TEST_PID_PHASE_POS_ACCEL = 0,
    APP_TEST_PID_PHASE_POS_HOLD,
    APP_TEST_PID_PHASE_POS_DECEL,
    APP_TEST_PID_PHASE_NEG_ACCEL,
    APP_TEST_PID_PHASE_NEG_HOLD,
    APP_TEST_PID_PHASE_NEG_DECEL
} App_TestPIDPhase_t;

volatile float app_test_motor_fl_rpm = 0.0f;
volatile float app_test_motor_fr_rpm = 0.0f;
volatile float app_test_motor_rl_rpm = 0.0f;
volatile float app_test_motor_rr_rpm = 0.0f;

volatile float app_test_motor_fl_mps = 0.0f;
volatile float app_test_motor_fr_mps = 0.0f;
volatile float app_test_motor_rl_mps = 0.0f;
volatile float app_test_motor_rr_mps = 0.0f;

volatile int32_t app_test_motor_fl_cnt_per_s = 0;
volatile int32_t app_test_motor_fr_cnt_per_s = 0;
volatile int32_t app_test_motor_rl_cnt_per_s = 0;
volatile int32_t app_test_motor_rr_cnt_per_s = 0;

volatile uint32_t app_test_motor_speed_update_cnt = 0u;

volatile float app_test_motor_fl_output_percent = 0.0f;
volatile float app_test_motor_fr_output_percent = 0.0f;
volatile float app_test_motor_rl_output_percent = 0.0f;
volatile float app_test_motor_rr_output_percent = 0.0f;

volatile float app_test_motor_fl_target_rpm = 0.0f;
volatile float app_test_motor_fr_target_rpm = 0.0f;
volatile float app_test_motor_rl_target_rpm = 0.0f;
volatile float app_test_motor_rr_target_rpm = 0.0f;

volatile float app_test_chassis_target_vx_mps = 0.0f;
volatile float app_test_chassis_target_vy_mps = 0.0f;
volatile float app_test_chassis_vx_mps = 0.0f;
volatile float app_test_chassis_vy_mps = 0.0f;
volatile float app_test_chassis_wz_radps = 0.0f;

volatile uint8_t app_test_motor_current = MOTOR_FL;
volatile uint8_t app_test_motor_phase = APP_TEST_PHASE_POS_ACCEL;
volatile uint16_t app_test_motor_phase_tick = 0u;
volatile uint8_t app_test_motor_sequence_done = 0u;

volatile uint8_t app_test_motor_pid_phase = APP_TEST_PID_PHASE_POS_ACCEL;
volatile uint16_t app_test_motor_pid_phase_tick = 0u;
volatile uint32_t app_test_motor_pid_cycle_cnt = 0u;

static const MotorId_t app_test_motor_order[MOTOR_NUM] = {
    MOTOR_FL,
    MOTOR_FR,
    MOTOR_RL,
    MOTOR_RR,
};

static uint8_t app_test_motor_order_index = 0u;
static uint16_t app_test_motor_local_phase_tick = 0u;
static App_TestMotorPhase_t app_test_motor_local_phase = APP_TEST_PHASE_POS_ACCEL;

static PID_t app_test_motor_pid[MOTOR_NUM];
static App_TestPIDPhase_t app_test_motor_pid_local_phase = APP_TEST_PID_PHASE_POS_ACCEL;
static uint16_t app_test_motor_pid_local_phase_tick = 0u;
static uint32_t app_test_motor_pid_local_cycle_cnt = 0u;

static float App_TestGetRampRatio(uint16_t tick)
{
    if (APP_TEST_MOTOR_RAMP_TICKS <= 1u)
    {
        return 1.0f;
    }

    if (tick >= (APP_TEST_MOTOR_RAMP_TICKS - 1u))
    {
        return 1.0f;
    }

    return (float)tick / (float)(APP_TEST_MOTOR_RAMP_TICKS - 1u);
}

static float App_TestGetPIDRampRatio(uint16_t tick)
{
    if (APP_TEST_PID_RAMP_TICKS <= 1u)
    {
        return 1.0f;
    }

    if (tick >= (APP_TEST_PID_RAMP_TICKS - 1u))
    {
        return 1.0f;
    }

    return (float)tick / (float)(APP_TEST_PID_RAMP_TICKS - 1u);
}

static void App_TestClearOutputPercentDebug(void)
{
    app_test_motor_fl_output_percent = 0.0f;
    app_test_motor_fr_output_percent = 0.0f;
    app_test_motor_rl_output_percent = 0.0f;
    app_test_motor_rr_output_percent = 0.0f;
}

static void App_TestClearTargetRPMDebug(void)
{
    app_test_motor_fl_target_rpm = 0.0f;
    app_test_motor_fr_target_rpm = 0.0f;
    app_test_motor_rl_target_rpm = 0.0f;
    app_test_motor_rr_target_rpm = 0.0f;

    app_test_chassis_target_vx_mps = 0.0f;
    app_test_chassis_target_vy_mps = 0.0f;
}

static float App_TestRPMToWheelMPS(float rpm)
{
    return rpm * APP_TEST_RPM_TO_RADPS * CHASSIS_MECANUM_WHEEL_RADIUS_M;
}

static void App_TestUpdateChassisSpeedMPSDebug(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm)
{
    float fl_radps = fl_rpm * APP_TEST_RPM_TO_RADPS;
    float fr_radps = fr_rpm * APP_TEST_RPM_TO_RADPS;
    float rl_radps = rl_rpm * APP_TEST_RPM_TO_RADPS;
    float rr_radps = rr_rpm * APP_TEST_RPM_TO_RADPS;
    float radius = CHASSIS_MECANUM_WHEEL_RADIUS_M;
    float wheel_base = CHASSIS_MECANUM_HALF_LENGTH_M + CHASSIS_MECANUM_HALF_WIDTH_M;

    app_test_motor_fl_mps = App_TestRPMToWheelMPS(fl_rpm);
    app_test_motor_fr_mps = App_TestRPMToWheelMPS(fr_rpm);
    app_test_motor_rl_mps = App_TestRPMToWheelMPS(rl_rpm);
    app_test_motor_rr_mps = App_TestRPMToWheelMPS(rr_rpm);

    app_test_chassis_vx_mps = radius * (fl_radps + fr_radps + rl_radps + rr_radps) * 0.25f;
    app_test_chassis_vy_mps = radius * (-fl_radps + fr_radps + rl_radps - rr_radps) * 0.25f;

    if (wheel_base > 0.0f)
    {
        app_test_chassis_wz_radps = radius * (-fl_radps + fr_radps - rl_radps + rr_radps) / (4.0f * wheel_base);
    }
    else
    {
        app_test_chassis_wz_radps = 0.0f;
    }
}

static void App_TestSetAllTargetRPMDebug(float target_rpm)
{
    app_test_motor_fl_target_rpm = target_rpm;
    app_test_motor_fr_target_rpm = target_rpm;
    app_test_motor_rl_target_rpm = target_rpm;
    app_test_motor_rr_target_rpm = target_rpm;

    app_test_chassis_target_vx_mps = App_TestRPMToWheelMPS(target_rpm);
    app_test_chassis_target_vy_mps = 0.0f;
}

void App_TestUpdateChassisTargetRPMDebug(void)
{
    static uint32_t last_chassis_update_cnt = 0u;
    AppChasisControlState_t state = App_ChasisGetControlState();

    if (state.update_cnt == last_chassis_update_cnt)
    {
        return;
    }

    last_chassis_update_cnt = state.update_cnt;

    app_test_motor_fl_target_rpm = state.target_rpm.fl_rpm;
    app_test_motor_fr_target_rpm = state.target_rpm.fr_rpm;
    app_test_motor_rl_target_rpm = state.target_rpm.rl_rpm;
    app_test_motor_rr_target_rpm = state.target_rpm.rr_rpm;

    app_test_chassis_target_vx_mps = state.cmd.vx_mps;
    app_test_chassis_target_vy_mps = state.cmd.vy_mps;

    app_test_motor_fl_output_percent = state.output_percent.fl_percent;
    app_test_motor_fr_output_percent = state.output_percent.fr_percent;
    app_test_motor_rl_output_percent = state.output_percent.rl_percent;
    app_test_motor_rr_output_percent = state.output_percent.rr_percent;
}

static void App_TestUpdateSpeedDebug(void)
{
    app_test_motor_fl_rpm = Motor_GetSpeedRPM(MOTOR_FL);
    app_test_motor_fr_rpm = Motor_GetSpeedRPM(MOTOR_FR);
    app_test_motor_rl_rpm = Motor_GetSpeedRPM(MOTOR_RL);
    app_test_motor_rr_rpm = Motor_GetSpeedRPM(MOTOR_RR);

    app_test_motor_fl_cnt_per_s = Motor_GetSpeedCntPerS(MOTOR_FL);
    app_test_motor_fr_cnt_per_s = Motor_GetSpeedCntPerS(MOTOR_FR);
    app_test_motor_rl_cnt_per_s = Motor_GetSpeedCntPerS(MOTOR_RL);
    app_test_motor_rr_cnt_per_s = Motor_GetSpeedCntPerS(MOTOR_RR);

    App_TestUpdateChassisSpeedMPSDebug(app_test_motor_fl_rpm,
                                       app_test_motor_fr_rpm,
                                       app_test_motor_rl_rpm,
                                       app_test_motor_rr_rpm);
}

static void App_TestSetOutputPercentDebug(MotorId_t motor, float output_percent)
{
    App_TestClearOutputPercentDebug();

    switch (motor)
    {
        case MOTOR_FL:
            app_test_motor_fl_output_percent = output_percent;
            break;

        case MOTOR_FR:
            app_test_motor_fr_output_percent = output_percent;
            break;

        case MOTOR_RL:
            app_test_motor_rl_output_percent = output_percent;
            break;

        case MOTOR_RR:
            app_test_motor_rr_output_percent = output_percent;
            break;

        default:
            break;
    }
}

static void App_TestAdvancePhase(void)
{
    if (app_test_motor_local_phase == APP_TEST_PHASE_BRAKE)
    {
        if ((app_test_motor_order_index + 1u) >= MOTOR_NUM)
        {
            app_test_motor_local_phase = APP_TEST_PHASE_DONE;
            app_test_motor_sequence_done = 1u;
            return;
        }

        app_test_motor_order_index++;
        app_test_motor_local_phase_tick = 0u;
        app_test_motor_local_phase = APP_TEST_PHASE_POS_ACCEL;
        return;
    }

    app_test_motor_local_phase_tick++;

    if (app_test_motor_local_phase_tick < APP_TEST_MOTOR_RAMP_TICKS)
    {
        return;
    }

    app_test_motor_local_phase_tick = 0u;
    app_test_motor_local_phase = (App_TestMotorPhase_t)((uint8_t)app_test_motor_local_phase + 1u);
}

static void App_TestUpdateMotorSequence(void)
{
    MotorId_t motor;
    float ratio;
    float output_percent = 0.0f;

    if (app_test_motor_sequence_done != 0u)
    {
        return;
    }

    motor = app_test_motor_order[app_test_motor_order_index];
    ratio = App_TestGetRampRatio(app_test_motor_local_phase_tick);

    switch (app_test_motor_local_phase)
    {
        case APP_TEST_PHASE_POS_ACCEL:
            output_percent = APP_TEST_MOTOR_MAX_OUTPUT_PERCENT * ratio;
            Motor_SetOutputPercent(motor, output_percent);
            break;

        case APP_TEST_PHASE_POS_DECEL:
            output_percent = APP_TEST_MOTOR_MAX_OUTPUT_PERCENT * (1.0f - ratio);
            Motor_SetOutputPercent(motor, output_percent);
            break;

        case APP_TEST_PHASE_NEG_ACCEL:
            output_percent = -APP_TEST_MOTOR_MAX_OUTPUT_PERCENT * ratio;
            Motor_SetOutputPercent(motor, output_percent);
            break;

        case APP_TEST_PHASE_NEG_DECEL:
            output_percent = -APP_TEST_MOTOR_MAX_OUTPUT_PERCENT * (1.0f - ratio);
            Motor_SetOutputPercent(motor, output_percent);
            break;

        case APP_TEST_PHASE_BRAKE:
            output_percent = 0.0f;
            Motor_Brake(motor);
            break;

        case APP_TEST_PHASE_DONE:
        default:
            app_test_motor_sequence_done = 1u;
            output_percent = 0.0f;
            break;
    }

    app_test_motor_current = (uint8_t)motor;
    app_test_motor_phase = (uint8_t)app_test_motor_local_phase;
    app_test_motor_phase_tick = app_test_motor_local_phase_tick;
    App_TestSetOutputPercentDebug(motor, output_percent);

    App_TestAdvancePhase();
}

static float App_TestGetPIDTargetRPM(void)
{
    float ratio = App_TestGetPIDRampRatio(app_test_motor_pid_local_phase_tick);

    switch (app_test_motor_pid_local_phase)
    {
        case APP_TEST_PID_PHASE_POS_ACCEL:
            return APP_TEST_PID_TARGET_RPM * ratio;

        case APP_TEST_PID_PHASE_POS_HOLD:
            return APP_TEST_PID_TARGET_RPM;

        case APP_TEST_PID_PHASE_POS_DECEL:
            return APP_TEST_PID_TARGET_RPM * (1.0f - ratio);

        case APP_TEST_PID_PHASE_NEG_ACCEL:
            return -APP_TEST_PID_TARGET_RPM * ratio;

        case APP_TEST_PID_PHASE_NEG_HOLD:
            return -APP_TEST_PID_TARGET_RPM;

        case APP_TEST_PID_PHASE_NEG_DECEL:
        default:
            return -APP_TEST_PID_TARGET_RPM * (1.0f - ratio);
    }
}

static uint16_t App_TestGetPIDPhaseTicks(App_TestPIDPhase_t phase)
{
    if ((phase == APP_TEST_PID_PHASE_POS_HOLD) || (phase == APP_TEST_PID_PHASE_NEG_HOLD))
    {
        return APP_TEST_PID_HOLD_TICKS;
    }

    return APP_TEST_PID_RAMP_TICKS;
}

static void App_TestAdvancePIDPhase(void)
{
    app_test_motor_pid_local_phase_tick++;

    if (app_test_motor_pid_local_phase_tick < App_TestGetPIDPhaseTicks(app_test_motor_pid_local_phase))
    {
        return;
    }

    app_test_motor_pid_local_phase_tick = 0u;

    if (app_test_motor_pid_local_phase == APP_TEST_PID_PHASE_NEG_DECEL)
    {
        app_test_motor_pid_local_phase = APP_TEST_PID_PHASE_POS_ACCEL;
        app_test_motor_pid_local_cycle_cnt++;
        return;
    }

    app_test_motor_pid_local_phase = (App_TestPIDPhase_t)((uint8_t)app_test_motor_pid_local_phase + 1u);
}

static void App_TestSetAllOutputPercentDebug(float fl, float fr, float rl, float rr)
{
    app_test_motor_fl_output_percent = fl;
    app_test_motor_fr_output_percent = fr;
    app_test_motor_rl_output_percent = rl;
    app_test_motor_rr_output_percent = rr;
}

static void App_TestApplyEzTunerSpeedPID(void)
{
    float max_output = eztuner_chassis_speed_max_output_percent;
    float integral_limit = eztuner_chassis_speed_integral_limit;

    if (max_output <= 0.0f)
    {
        max_output = 100.0f;
    }

    if (integral_limit < 0.0f)
    {
        integral_limit = -integral_limit;
    }

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_SetParam(&app_test_motor_pid[i],
                     eztuner_chassis_speed_kp,
                     eztuner_chassis_speed_ki,
                     eztuner_chassis_speed_kd);
        PID_SetOutputLimit(&app_test_motor_pid[i], -max_output, max_output);
        PID_SetIntegralLimit(&app_test_motor_pid[i], -integral_limit, integral_limit);
    }
}

static void App_TestUpdateMotorPID(void)
{
    float target_rpm = App_TestGetPIDTargetRPM();
    float fl_output = PID_Calc(&app_test_motor_pid[MOTOR_FL], target_rpm, app_test_motor_fl_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    float fr_output = PID_Calc(&app_test_motor_pid[MOTOR_FR], target_rpm, app_test_motor_fr_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    float rl_output = PID_Calc(&app_test_motor_pid[MOTOR_RL], target_rpm, app_test_motor_rl_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    float rr_output = PID_Calc(&app_test_motor_pid[MOTOR_RR], target_rpm, app_test_motor_rr_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);

    Motor_SetOutputPercent(MOTOR_FL, fl_output);
    Motor_SetOutputPercent(MOTOR_FR, fr_output);
    Motor_SetOutputPercent(MOTOR_RL, rl_output);
    Motor_SetOutputPercent(MOTOR_RR, rr_output);

    App_TestSetAllTargetRPMDebug(target_rpm);
    App_TestSetAllOutputPercentDebug(fl_output, fr_output, rl_output, rr_output);

    app_test_motor_pid_phase = (uint8_t)app_test_motor_pid_local_phase;
    app_test_motor_pid_phase_tick = app_test_motor_pid_local_phase_tick;
    app_test_motor_pid_cycle_cnt = app_test_motor_pid_local_cycle_cnt;

    App_TestAdvancePIDPhase();
}

static void App_TestUpdateMotorPID70PercentForward(void)
{
    float target_rpm = APP_TEST_PID_TUNE_TARGET_RPM;
    float fl_output;
    float fr_output;
    float rl_output;
    float rr_output;

    App_TestApplyEzTunerSpeedPID();

    fl_output = PID_Calc(&app_test_motor_pid[MOTOR_FL], target_rpm, app_test_motor_fl_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    fr_output = PID_Calc(&app_test_motor_pid[MOTOR_FR], target_rpm, app_test_motor_fr_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    rl_output = PID_Calc(&app_test_motor_pid[MOTOR_RL], target_rpm, app_test_motor_rl_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);
    rr_output = PID_Calc(&app_test_motor_pid[MOTOR_RR], target_rpm, app_test_motor_rr_rpm, APP_TEST_MOTOR_CONTROL_PERIOD_S);

    Motor_SetOutputPercent(MOTOR_FL, fl_output);
    Motor_SetOutputPercent(MOTOR_FR, fr_output);
    Motor_SetOutputPercent(MOTOR_RL, rl_output);
    Motor_SetOutputPercent(MOTOR_RR, rr_output);

    App_TestSetAllTargetRPMDebug(target_rpm);
    App_TestSetAllOutputPercentDebug(fl_output, fr_output, rl_output, rr_output);

    app_test_motor_pid_phase = APP_TEST_PID_PHASE_POS_HOLD;
    app_test_motor_pid_phase_tick = 0u;
    app_test_motor_pid_cycle_cnt++;
}

static void App_TestMotorSpeed100HzCallback(void)
{
    Motor_UpdateSpeed(APP_TEST_MOTOR_SPEED_PERIOD_MS);
    App_TestUpdateMotorSequence();
    App_TestUpdateSpeedDebug();

    app_test_motor_speed_update_cnt++;
}

static void App_TestMotorPID100HzCallback(void)
{
    Motor_UpdateSpeed(APP_TEST_MOTOR_SPEED_PERIOD_MS);
    App_TestUpdateSpeedDebug();
    App_TestUpdateMotorPID();

    app_test_motor_speed_update_cnt++;
}

static void App_TestMotorPID70PercentForward100HzCallback(void)
{
    Motor_UpdateSpeed(APP_TEST_MOTOR_SPEED_PERIOD_MS);
    App_TestUpdateSpeedDebug();
    App_TestUpdateMotorPID70PercentForward();

    app_test_motor_speed_update_cnt++;
}

void App_TestMotorSequenceStart(void)
{
    Motor_Init();
    Motor_StopAll();

    app_test_motor_speed_update_cnt = 0u;
    app_test_motor_order_index = 0u;
    app_test_motor_local_phase_tick = 0u;
    app_test_motor_local_phase = APP_TEST_PHASE_POS_ACCEL;
    app_test_motor_current = MOTOR_FL;
    app_test_motor_phase = APP_TEST_PHASE_POS_ACCEL;
    app_test_motor_phase_tick = 0u;
    app_test_motor_sequence_done = 0u;
    App_TestClearOutputPercentDebug();
    App_TestClearTargetRPMDebug();

    BSP_Timer100Hz_RegisterCallback(App_TestMotorSpeed100HzCallback);
    BSP_Timer100Hz_Init();
}

void App_TestMotorPIDStart(void)
{
    Motor_Init();
    Motor_StopAll();

    app_test_motor_speed_update_cnt = 0u;
    app_test_motor_pid_local_phase = APP_TEST_PID_PHASE_POS_ACCEL;
    app_test_motor_pid_local_phase_tick = 0u;
    app_test_motor_pid_local_cycle_cnt = 0u;
    app_test_motor_pid_phase = APP_TEST_PID_PHASE_POS_ACCEL;
    app_test_motor_pid_phase_tick = 0u;
    app_test_motor_pid_cycle_cnt = 0u;

    App_TestClearOutputPercentDebug();
    App_TestClearTargetRPMDebug();

    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        PID_Init(&app_test_motor_pid[i], APP_TEST_PID_KP, APP_TEST_PID_KI, APP_TEST_PID_KD);
        PID_SetOutputLimit(&app_test_motor_pid[i], -100.0f, 100.0f);
        PID_SetIntegralLimit(&app_test_motor_pid[i], -APP_TEST_PID_INTEGRAL_LIMIT, APP_TEST_PID_INTEGRAL_LIMIT);
    }

    BSP_Timer100Hz_RegisterCallback(App_TestMotorPID100HzCallback);
    BSP_Timer100Hz_Init();
}

void App_TestMotorPID70PercentForwardStart(void)
{
    Motor_Init();
    Motor_StopAll();

    app_test_motor_speed_update_cnt = 0u;
    app_test_motor_pid_local_phase = APP_TEST_PID_PHASE_POS_HOLD;
    app_test_motor_pid_local_phase_tick = 0u;
    app_test_motor_pid_local_cycle_cnt = 0u;
    app_test_motor_pid_phase = APP_TEST_PID_PHASE_POS_HOLD;
    app_test_motor_pid_phase_tick = 0u;
    app_test_motor_pid_cycle_cnt = 0u;

    App_TestClearOutputPercentDebug();
    App_TestSetAllTargetRPMDebug(APP_TEST_PID_TUNE_TARGET_RPM);

    for (uint8_t i = 0u; i < MOTOR_NUM; i++)
    {
        PID_Init(&app_test_motor_pid[i],
                 eztuner_chassis_speed_kp,
                 eztuner_chassis_speed_ki,
                 eztuner_chassis_speed_kd);
    }

    App_TestApplyEzTunerSpeedPID();

    BSP_Timer100Hz_RegisterCallback(App_TestMotorPID70PercentForward100HzCallback);
    BSP_Timer100Hz_Init();
}

void App_TestMotorSpeedStart(void)
{
    App_TestMotorSequenceStart();
}
