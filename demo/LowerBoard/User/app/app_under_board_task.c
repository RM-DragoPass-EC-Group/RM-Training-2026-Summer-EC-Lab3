#include "app_under_board_task.h"

#include "app_chasis.h"
#include "app_connectivity.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_timer.h"
#include "drv/drv_motor.h"

#define APP_UNDER_BOARD_TICKS_FROM_MS(ms) \
    (((uint32_t)(ms) + APP_UNDER_BOARD_TASK_PERIOD_MS - 1u) / APP_UNDER_BOARD_TASK_PERIOD_MS)

#define APP_UNDER_BOARD_TIMEOUT_TICKS APP_UNDER_BOARD_TICKS_FROM_MS(APP_UNDER_BOARD_UART_TIMEOUT_MS)
#define APP_UNDER_BOARD_LED_TOGGLE_TICKS APP_UNDER_BOARD_TICKS_FROM_MS(APP_UNDER_BOARD_LED_TOGGLE_MS)

volatile uint8_t app_under_board_task_100hz_flag = 0u;
volatile uint32_t app_under_board_task_tick = 0u;
volatile uint8_t app_under_board_task_online = 0u;
volatile uint32_t app_under_board_task_timeout_count = 0u;

static AppConnectivityChassisCommand_t app_under_board_expected_cmd = {
    APP_CONNECTIVITY_CMD_COAST,
    {{0.0f, 0.0f, 0.0f}},
};
static uint32_t app_under_board_last_rx_tick = 0u;
static uint32_t app_under_board_led_tick = 0u;
static uint8_t app_under_board_output_enabled = 0u;

static void App_UnderBoardTask100HzCallback(void)
{
    app_under_board_task_100hz_flag = 1u;
}

static void App_UnderBoardTaskUpdateLED(void)
{
    app_under_board_led_tick++;

    if (app_under_board_led_tick < APP_UNDER_BOARD_LED_TOGGLE_TICKS)
    {
        return;
    }

    app_under_board_led_tick = 0u;
    BSP_LED_Toggle();
}

static void App_UnderBoardTaskUpdateRxCommand(void)
{
    AppConnectivityChassisCommand_t cmd;

    if (App_ConnectivityFetchChassisCommand(&cmd) == 0u)
    {
        return;
    }

    app_under_board_last_rx_tick = app_under_board_task_tick;

    if (app_under_board_task_online == 0u)
    {
        App_ChasisSpeedPIDReset();
    }

    app_under_board_task_online = 1u;

    switch (cmd.type)
    {
        case APP_CONNECTIVITY_CMD_COAST:
            app_under_board_expected_cmd.type = APP_CONNECTIVITY_CMD_COAST;
            App_ChasisStop();
            app_under_board_output_enabled = 0u;
            break;

        case APP_CONNECTIVITY_CMD_BRAKE:
            app_under_board_expected_cmd.type = APP_CONNECTIVITY_CMD_BRAKE;
            App_ChasisBrake();
            app_under_board_output_enabled = 0u;
            break;

        case APP_CONNECTIVITY_CMD_VELOCITY:
            app_under_board_expected_cmd.type = APP_CONNECTIVITY_CMD_VELOCITY;
            app_under_board_expected_cmd.data.velocity = cmd.data.velocity;
            app_under_board_output_enabled = 1u;
            break;

        case APP_CONNECTIVITY_CMD_HEADING:
            app_under_board_expected_cmd.type = APP_CONNECTIVITY_CMD_HEADING;
            app_under_board_expected_cmd.data.heading = cmd.data.heading;
            app_under_board_output_enabled = 1u;
            break;

        case APP_CONNECTIVITY_CMD_NONE:
        default:
            break;
    }
}

static uint8_t App_UnderBoardTaskIsCommandTimeout(void)
{
    if (app_under_board_task_online == 0u)
    {
        return 1u;
    }

    return ((app_under_board_task_tick - app_under_board_last_rx_tick) > APP_UNDER_BOARD_TIMEOUT_TICKS) ? 1u : 0u;
}

static void App_UnderBoardTaskDisableOutput(void)
{
    if (app_under_board_output_enabled == 0u)
    {
        return;
    }

    App_ChasisStop();
    app_under_board_output_enabled = 0u;
    app_under_board_task_online = 0u;
    app_under_board_task_timeout_count++;
}

static void App_UnderBoardTaskUpdateChassis(void)
{
    if (App_UnderBoardTaskIsCommandTimeout())
    {
        App_UnderBoardTaskDisableOutput();
        Motor_UpdateSpeed(APP_UNDER_BOARD_TASK_PERIOD_MS);
        return;
    }

    if (app_under_board_output_enabled == 0u)
    {
        Motor_UpdateSpeed(APP_UNDER_BOARD_TASK_PERIOD_MS);
        return;
    }

    if (app_under_board_expected_cmd.type == APP_CONNECTIVITY_CMD_VELOCITY)
    {
        App_ChasisVelocityControl(app_under_board_expected_cmd.data.velocity,
                                  APP_UNDER_BOARD_TASK_PERIOD_MS);
        return;
    }

    if (app_under_board_expected_cmd.type == APP_CONNECTIVITY_CMD_HEADING)
    {
        App_ChasisHeadingControl(app_under_board_expected_cmd.data.heading,
                                 APP_UNDER_BOARD_TASK_PERIOD_MS);
    }
}

void App_UnderBoardTaskInit(void)
{
    app_under_board_task_100hz_flag = 0u;
    app_under_board_task_tick = 0u;
    app_under_board_task_online = 0u;
    app_under_board_task_timeout_count = 0u;
    app_under_board_last_rx_tick = 0u;
    app_under_board_led_tick = 0u;
    app_under_board_output_enabled = 0u;

    app_under_board_expected_cmd.type = APP_CONNECTIVITY_CMD_COAST;
    app_under_board_expected_cmd.data.velocity.vx_mps = 0.0f;
    app_under_board_expected_cmd.data.velocity.vy_mps = 0.0f;
    app_under_board_expected_cmd.data.velocity.wz_radps = 0.0f;

    BSP_LED_Off();
    App_ChasisInit();
    App_ChasisStop();
    App_ConnectivityInit();

    BSP_Timer100Hz_RegisterCallback(App_UnderBoardTask100HzCallback);
    (void)BSP_Timer100Hz_Init();
}

void App_UnderBoardTaskRun(void)
{
    if (app_under_board_task_100hz_flag == 0u)
    {
        return;
    }

    app_under_board_task_100hz_flag = 0u;
    app_under_board_task_tick++;

    App_UnderBoardTaskUpdateLED();
    App_UnderBoardTaskUpdateRxCommand();
    App_UnderBoardTaskUpdateChassis();
    (void)App_ConnectivitySendChassisFeedback(App_ChasisGetControlState());
}
