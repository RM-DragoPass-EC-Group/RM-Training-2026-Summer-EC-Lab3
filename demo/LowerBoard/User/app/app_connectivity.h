/**
 * @file app_connectivity.h
 * @brief App-level UART connectivity command interface.
 */

#ifndef __APP_CONNECTIVITY_H__
#define __APP_CONNECTIVITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_chasis.h"
#include "main.h"

typedef enum
{
    APP_CONNECTIVITY_CMD_NONE = 0,
    APP_CONNECTIVITY_CMD_COAST,
    APP_CONNECTIVITY_CMD_BRAKE,
    APP_CONNECTIVITY_CMD_VELOCITY,
    APP_CONNECTIVITY_CMD_HEADING,
} AppConnectivityCommandType_t;

typedef struct
{
    AppConnectivityCommandType_t type;
    union
    {
        ChassisMecanumCmd_t velocity;
        AppChasisHeadingCmd_t heading;
    } data;
} AppConnectivityChassisCommand_t;

extern volatile AppConnectivityChassisCommand_t app_connectivity_rx_command;
extern volatile uint8_t app_connectivity_rx_ready;
extern volatile uint32_t app_connectivity_rx_count;
extern volatile uint32_t app_connectivity_rx_error_count;
extern volatile uint32_t app_connectivity_uart_error_count;
extern volatile uint8_t app_connectivity_tx_busy;
extern volatile uint8_t app_connectivity_loopback_enabled;
extern volatile uint32_t app_connectivity_tx_count;
extern volatile uint32_t app_connectivity_tx_error_count;

void App_ConnectivityInit(void);
uint8_t App_ConnectivityIsRxReady(void);
uint8_t App_ConnectivityFetchChassisCommand(AppConnectivityChassisCommand_t *cmd);
void App_ConnectivitySetLoopback(uint8_t enable);
HAL_StatusTypeDef App_ConnectivitySendFrame(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len);
HAL_StatusTypeDef App_ConnectivitySendChassisCommand(const AppConnectivityChassisCommand_t *cmd);
HAL_StatusTypeDef App_ConnectivitySendChassisFeedback(AppChasisControlState_t state);

#ifdef __cplusplus
}
#endif

#endif
