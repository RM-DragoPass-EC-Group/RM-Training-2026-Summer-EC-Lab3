/**
 * @file bsp_uart.h
 * @brief BSP UART interface based on CubeMX-configured USART6.
 *
 * This module wraps USART6 transmit/receive APIs and lets user code register
 * callbacks dispatched from HAL UART interrupt/DMA callbacks.
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief UART channel index.
 */
typedef enum
{
    /** CubeMX-configured USART6. */
    BSP_UART_USART6 = 0,
    /** Number of UART channels. */
    BSP_UART_NUM
} BSP_UARTId_t;

/**
 * @brief UART receive complete callback.
 *
 * @param uart UART channel.
 * @param data Receive buffer pointer passed to BSP_UART_Receive*().
 * @param len Completed receive length in bytes.
 */
typedef void (*BSP_UARTRxCallback_t)(BSP_UARTId_t uart, uint8_t *data, uint16_t len);

/**
 * @brief UART transmit complete callback.
 *
 * @param uart UART channel.
 */
typedef void (*BSP_UARTTxCallback_t)(BSP_UARTId_t uart);

/**
 * @brief UART error callback.
 *
 * @param uart UART channel.
 * @param error_code HAL UART error flags.
 */
typedef void (*BSP_UARTErrorCallback_t)(BSP_UARTId_t uart, uint32_t error_code);

/**
 * @brief Initialize BSP UART runtime state.
 *
 * Call this after MX_USART6_UART_Init(). CubeMX owns the hardware
 * configuration, so this function only clears local BSP state.
 */
void BSP_UART_Init(void);

/**
 * @brief Register receive complete callback.
 *
 * Passing NULL unregisters the current callback.
 *
 * @param uart UART channel.
 * @param callback User callback.
 */
void BSP_UART_RegisterRxCallback(BSP_UARTId_t uart, BSP_UARTRxCallback_t callback);

/**
 * @brief Register transmit complete callback.
 *
 * Passing NULL unregisters the current callback.
 *
 * @param uart UART channel.
 * @param callback User callback.
 */
void BSP_UART_RegisterTxCallback(BSP_UARTId_t uart, BSP_UARTTxCallback_t callback);

/**
 * @brief Register UART error callback.
 *
 * Passing NULL unregisters the current callback.
 *
 * @param uart UART channel.
 * @param callback User callback.
 */
void BSP_UART_RegisterErrorCallback(BSP_UARTId_t uart, BSP_UARTErrorCallback_t callback);

/**
 * @brief Send bytes using blocking UART transmit.
 *
 * @param uart UART channel.
 * @param data Data buffer.
 * @param len Data length in bytes.
 * @param timeout_ms HAL timeout in milliseconds.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_Send(BSP_UARTId_t uart, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief Send bytes using UART interrupt transmit.
 *
 * The registered TX callback is called after completion.
 *
 * @param uart UART channel.
 * @param data Data buffer. Keep it valid until TX completes.
 * @param len Data length in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_SendIT(BSP_UARTId_t uart, const uint8_t *data, uint16_t len);

/**
 * @brief Send bytes using UART DMA transmit.
 *
 * The registered TX callback is called after completion.
 *
 * @param uart UART channel.
 * @param data Data buffer. Keep it valid until TX completes.
 * @param len Data length in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_SendDMA(BSP_UARTId_t uart, const uint8_t *data, uint16_t len);

/**
 * @brief Receive a fixed number of bytes using UART interrupt receive.
 *
 * The registered RX callback is called after len bytes are received.
 *
 * @param uart UART channel.
 * @param data Receive buffer. Keep it valid until RX completes.
 * @param len Expected receive length in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_ReceiveIT(BSP_UARTId_t uart, uint8_t *data, uint16_t len);

/**
 * @brief Receive a fixed number of bytes using UART DMA receive.
 *
 * The registered RX callback is called after len bytes are received.
 *
 * @param uart UART channel.
 * @param data Receive buffer. Keep it valid until RX completes.
 * @param len Expected receive length in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_ReceiveDMA(BSP_UARTId_t uart, uint8_t *data, uint16_t len);

/**
 * @brief Receive bytes until expected length or UART idle using interrupt mode.
 *
 * The registered RX callback is called with the actual received length. This is
 * useful for variable-length packets from the upper computer.
 *
 * @param uart UART channel.
 * @param data Receive buffer. Keep it valid until RX event completes.
 * @param max_len Receive buffer size in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_ReceiveToIdleIT(BSP_UARTId_t uart, uint8_t *data, uint16_t max_len);

/**
 * @brief Receive bytes until expected length or UART idle using DMA mode.
 *
 * The registered RX callback is called with the actual received length. This is
 * useful for variable-length packets from the upper computer.
 *
 * @param uart UART channel.
 * @param data Receive buffer. Keep it valid until RX event completes.
 * @param max_len Receive buffer size in bytes.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_ReceiveToIdleDMA(BSP_UARTId_t uart, uint8_t *data, uint16_t max_len);

/**
 * @brief Abort ongoing UART receive.
 *
 * @param uart UART channel.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_AbortReceive(BSP_UARTId_t uart);

/**
 * @brief Abort ongoing UART transmit.
 *
 * @param uart UART channel.
 * @return HAL_OK on success, otherwise HAL error status.
 */
HAL_StatusTypeDef BSP_UART_AbortTransmit(BSP_UARTId_t uart);

#ifdef __cplusplus
}
#endif

#endif
