#include "app_connectivity.h"

#include "bsp/bsp_crc.h"
#include "bsp/bsp_uart.h"
#include "common/chassis_protocol.h"
#include "common/frame_codec.h"
#include "drv/drv_motor.h"
#include "modules/module_chassis_mecanum.h"

#include <string.h>

#define APP_CONNECTIVITY_RX_DMA_LEN  FRAME_CODEC_MAX_FRAME_LEN

volatile AppConnectivityChassisCommand_t app_connectivity_rx_command;
volatile uint8_t app_connectivity_rx_ready = 0u;
volatile uint32_t app_connectivity_rx_count = 0u;
volatile uint32_t app_connectivity_rx_error_count = 0u;
volatile uint32_t app_connectivity_uart_error_count = 0u;
volatile uint8_t app_connectivity_tx_busy = 0u;
volatile uint8_t app_connectivity_loopback_enabled = 0u;
volatile uint32_t app_connectivity_tx_count = 0u;
volatile uint32_t app_connectivity_tx_error_count = 0u;

static uint8_t app_connectivity_rx_dma_buf[APP_CONNECTIVITY_RX_DMA_LEN];
static uint8_t app_connectivity_tx_dma_buf[FRAME_CODEC_MAX_FRAME_LEN];
static uint8_t app_connectivity_decoder_buf[FRAME_CODEC_MAX_FRAME_LEN];
static frame_decoder_t app_connectivity_decoder;

static HAL_StatusTypeDef App_ConnectivityStartRx(void)
{
    return BSP_UART_ReceiveToIdleDMA(BSP_UART_USART6,
                                     app_connectivity_rx_dma_buf,
                                     APP_CONNECTIVITY_RX_DMA_LEN);
}

static HAL_StatusTypeDef App_ConnectivitySendBytesDMA(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0u) || (len > FRAME_CODEC_MAX_FRAME_LEN))
    {
        app_connectivity_tx_error_count++;
        return HAL_ERROR;
    }

    if (app_connectivity_tx_busy != 0u)
    {
        app_connectivity_tx_error_count++;
        return HAL_BUSY;
    }

    if (data != app_connectivity_tx_dma_buf)
    {
        (void)memcpy(app_connectivity_tx_dma_buf, data, len);
    }

    app_connectivity_tx_busy = 1u;

    status = BSP_UART_SendDMA(BSP_UART_USART6, app_connectivity_tx_dma_buf, len);
    if (status != HAL_OK)
    {
        app_connectivity_tx_busy = 0u;
        app_connectivity_tx_error_count++;
    }
    else
    {
        app_connectivity_tx_count++;
    }

    return status;
}

static void App_ConnectivityLoopbackFrame(const uint8_t *frame, uint16_t len)
{
    if (app_connectivity_loopback_enabled == 0u)
    {
        return;
    }

    (void)App_ConnectivitySendBytesDMA(frame, len);
}

static void App_ConnectivityClearCommand(void)
{
    (void)memset((void *)&app_connectivity_rx_command, 0, sizeof(app_connectivity_rx_command));
    app_connectivity_rx_ready = 0u;
}

static void App_ConnectivityStoreCoastCommand(void)
{
    app_connectivity_rx_command.type = APP_CONNECTIVITY_CMD_COAST;
    app_connectivity_rx_ready = 1u;
    app_connectivity_rx_count++;
}

static void App_ConnectivityStoreBrakeCommand(void)
{
    app_connectivity_rx_command.type = APP_CONNECTIVITY_CMD_BRAKE;
    app_connectivity_rx_ready = 1u;
    app_connectivity_rx_count++;
}

static void App_ConnectivityStoreVelocityCommand(const uint8_t *payload)
{
    chassis_velocity_cmd_t cmd;

    (void)memcpy(&cmd, payload, sizeof(cmd));

    app_connectivity_rx_command.type = APP_CONNECTIVITY_CMD_VELOCITY;
    app_connectivity_rx_command.data.velocity.vx_mps = cmd.vx_mps;
    app_connectivity_rx_command.data.velocity.vy_mps = cmd.vy_mps;
    app_connectivity_rx_command.data.velocity.wz_radps = cmd.wz_radps;
    app_connectivity_rx_ready = 1u;
    app_connectivity_rx_count++;
}

static void App_ConnectivityStoreHeadingCommand(const uint8_t *payload)
{
    chassis_heading_cmd_t cmd;

    (void)memcpy(&cmd, payload, sizeof(cmd));

    app_connectivity_rx_command.type = APP_CONNECTIVITY_CMD_HEADING;
    app_connectivity_rx_command.data.heading.vx_mps = cmd.vx_mps;
    app_connectivity_rx_command.data.heading.vy_mps = cmd.vy_mps;
    app_connectivity_rx_command.data.heading.target_yaw_rad = cmd.target_yaw_rad;
    app_connectivity_rx_command.data.heading.gyro_yaw_rad = cmd.gyro_yaw_rad;
    app_connectivity_rx_ready = 1u;
    app_connectivity_rx_count++;
}

static void App_ConnectivityHandleFrame(const uint8_t *frame, uint16_t len)
{
    uint8_t msg_id;
    uint8_t payload_len;
    const uint8_t *payload;
    uint8_t frame_valid = 0u;

    if ((frame == NULL) || (len < FRAME_CODEC_MIN_FRAME_LEN) || (frame[1] < FRAME_CODEC_MSG_ID_SIZE))
    {
        app_connectivity_rx_error_count++;
        return;
    }

    msg_id = frame[2];
    payload_len = (uint8_t)(frame[1] - FRAME_CODEC_MSG_ID_SIZE);
    payload = &frame[3];

    switch (msg_id)
    {
        case CHASSIS_MSG_ID_COAST:
            if (payload_len == 0U)
            {
                App_ConnectivityStoreCoastCommand();
                frame_valid = 1u;
            }
            break;

        case CHASSIS_MSG_ID_BRAKE:
            if (payload_len == 0U)
            {
                App_ConnectivityStoreBrakeCommand();
                frame_valid = 1u;
            }
            break;

        case CHASSIS_MSG_ID_VELOCITY:
            if (payload_len == sizeof(chassis_velocity_cmd_t))
            {
                App_ConnectivityStoreVelocityCommand(payload);
                frame_valid = 1u;
            }
            break;

        case CHASSIS_MSG_ID_HEADING:
            if (payload_len == sizeof(chassis_heading_cmd_t))
            {
                App_ConnectivityStoreHeadingCommand(payload);
                frame_valid = 1u;
            }
            break;

        default:
            break;
    }

    if (frame_valid != 0u)
    {
        App_ConnectivityLoopbackFrame(frame, len);
    }
    else
    {
        app_connectivity_rx_error_count++;
    }
}

static void App_ConnectivityRxCallback(BSP_UARTId_t uart, uint8_t *data, uint16_t len)
{
    (void)uart;

    if ((data != NULL) && (len != 0U))
    {
        FrameDecoder_Input(&app_connectivity_decoder, data, len);
    }

    (void)App_ConnectivityStartRx();
}

static void App_ConnectivityErrorCallback(BSP_UARTId_t uart, uint32_t error_code)
{
    (void)uart;
    (void)error_code;

    app_connectivity_uart_error_count++;
    app_connectivity_tx_busy = 0u;
    (void)BSP_UART_AbortReceive(BSP_UART_USART6);
    (void)BSP_UART_AbortTransmit(BSP_UART_USART6);
    FrameDecoder_Reset(&app_connectivity_decoder);
    (void)App_ConnectivityStartRx();
}

static void App_ConnectivityTxCallback(BSP_UARTId_t uart)
{
    (void)uart;

    app_connectivity_tx_busy = 0u;
}

void App_ConnectivityInit(void)
{
    BSP_UART_Init();
    BSP_CRC_Init();

    app_connectivity_rx_count = 0u;
    app_connectivity_rx_error_count = 0u;
    app_connectivity_uart_error_count = 0u;
    app_connectivity_tx_busy = 0u;
    app_connectivity_loopback_enabled = 0u;
    app_connectivity_tx_count = 0u;
    app_connectivity_tx_error_count = 0u;
    App_ConnectivityClearCommand();

    if (!FrameDecoder_Init(&app_connectivity_decoder,
                           app_connectivity_decoder_buf,
                           sizeof(app_connectivity_decoder_buf),
                           App_ConnectivityHandleFrame))
    {
        app_connectivity_rx_error_count++;
    }

    BSP_UART_RegisterRxCallback(BSP_UART_USART6, App_ConnectivityRxCallback);
    BSP_UART_RegisterTxCallback(BSP_UART_USART6, App_ConnectivityTxCallback);
    BSP_UART_RegisterErrorCallback(BSP_UART_USART6, App_ConnectivityErrorCallback);

    (void)App_ConnectivityStartRx();
}

uint8_t App_ConnectivityIsRxReady(void)
{
    return app_connectivity_rx_ready;
}

uint8_t App_ConnectivityFetchChassisCommand(AppConnectivityChassisCommand_t *cmd)
{
    if ((cmd == NULL) || (app_connectivity_rx_ready == 0u))
    {
        return 0u;
    }

    cmd->type = app_connectivity_rx_command.type;
    switch (cmd->type)
    {
        case APP_CONNECTIVITY_CMD_VELOCITY:
            cmd->data.velocity.vx_mps = app_connectivity_rx_command.data.velocity.vx_mps;
            cmd->data.velocity.vy_mps = app_connectivity_rx_command.data.velocity.vy_mps;
            cmd->data.velocity.wz_radps = app_connectivity_rx_command.data.velocity.wz_radps;
            break;

        case APP_CONNECTIVITY_CMD_HEADING:
            cmd->data.heading.vx_mps = app_connectivity_rx_command.data.heading.vx_mps;
            cmd->data.heading.vy_mps = app_connectivity_rx_command.data.heading.vy_mps;
            cmd->data.heading.target_yaw_rad = app_connectivity_rx_command.data.heading.target_yaw_rad;
            cmd->data.heading.gyro_yaw_rad = app_connectivity_rx_command.data.heading.gyro_yaw_rad;
            break;

        case APP_CONNECTIVITY_CMD_COAST:
        case APP_CONNECTIVITY_CMD_BRAKE:
        case APP_CONNECTIVITY_CMD_NONE:
        default:
            break;
    }

    app_connectivity_rx_ready = 0u;

    return 1u;
}

void App_ConnectivitySetLoopback(uint8_t enable)
{
    app_connectivity_loopback_enabled = (enable == 0u) ? 0u : 1u;
}

HAL_StatusTypeDef App_ConnectivitySendFrame(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len)
{
    uint16_t frame_len = 0u;

    if (app_connectivity_tx_busy != 0u)
    {
        app_connectivity_tx_error_count++;
        return HAL_BUSY;
    }

    if (!FrameCodec_Encode(msg_id,
                           payload,
                           payload_len,
                           app_connectivity_tx_dma_buf,
                           sizeof(app_connectivity_tx_dma_buf),
                           &frame_len))
    {
        app_connectivity_tx_error_count++;
        return HAL_ERROR;
    }

    return App_ConnectivitySendBytesDMA(app_connectivity_tx_dma_buf, frame_len);
}

HAL_StatusTypeDef App_ConnectivitySendChassisCommand(const AppConnectivityChassisCommand_t *cmd)
{
    chassis_velocity_cmd_t velocity;
    chassis_heading_cmd_t heading;

    if (cmd == NULL)
    {
        app_connectivity_tx_error_count++;
        return HAL_ERROR;
    }

    switch (cmd->type)
    {
        case APP_CONNECTIVITY_CMD_COAST:
            return App_ConnectivitySendFrame(CHASSIS_MSG_ID_COAST, NULL, 0u);

        case APP_CONNECTIVITY_CMD_BRAKE:
            return App_ConnectivitySendFrame(CHASSIS_MSG_ID_BRAKE, NULL, 0u);

        case APP_CONNECTIVITY_CMD_VELOCITY:
            velocity.vx_mps = cmd->data.velocity.vx_mps;
            velocity.vy_mps = cmd->data.velocity.vy_mps;
            velocity.wz_radps = cmd->data.velocity.wz_radps;
            return App_ConnectivitySendFrame(CHASSIS_MSG_ID_VELOCITY,
                                             (const uint8_t *)&velocity,
                                             (uint8_t)sizeof(velocity));

        case APP_CONNECTIVITY_CMD_HEADING:
            heading.vx_mps = cmd->data.heading.vx_mps;
            heading.vy_mps = cmd->data.heading.vy_mps;
            heading.target_yaw_rad = cmd->data.heading.target_yaw_rad;
            heading.gyro_yaw_rad = cmd->data.heading.gyro_yaw_rad;
            return App_ConnectivitySendFrame(CHASSIS_MSG_ID_HEADING,
                                             (const uint8_t *)&heading,
                                             (uint8_t)sizeof(heading));

        case APP_CONNECTIVITY_CMD_NONE:
        default:
            app_connectivity_tx_error_count++;
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef App_ConnectivitySendChassisFeedback(AppChasisControlState_t state)
{
    chassis_feedback_t feedback;
    ChassisMecanumWheelRPM_t wheel_rpm;
    ChassisMecanumCmd_t chassis_speed;

    if (app_connectivity_tx_busy != 0u)
    {
        return HAL_BUSY;
    }

    wheel_rpm.fl_rpm = Motor_GetSpeedRPM(MOTOR_FL);
    wheel_rpm.fr_rpm = Motor_GetSpeedRPM(MOTOR_FR);
    wheel_rpm.rl_rpm = Motor_GetSpeedRPM(MOTOR_RL);
    wheel_rpm.rr_rpm = Motor_GetSpeedRPM(MOTOR_RR);
    chassis_speed = ChassisMecanum_ForwardKinematics(wheel_rpm);

    feedback.vx_mps = chassis_speed.vx_mps;
    feedback.vy_mps = chassis_speed.vy_mps;
    feedback.wz_radps = chassis_speed.wz_radps;
    feedback.fl_rpm = wheel_rpm.fl_rpm;
    feedback.fr_rpm = wheel_rpm.fr_rpm;
    feedback.rl_rpm = wheel_rpm.rl_rpm;
    feedback.rr_rpm = wheel_rpm.rr_rpm;
    feedback.update_cnt = state.update_cnt;

    return App_ConnectivitySendFrame(CHASSIS_MSG_ID_FEEDBACK,
                                     (const uint8_t *)&feedback,
                                     (uint8_t)sizeof(feedback));
}
