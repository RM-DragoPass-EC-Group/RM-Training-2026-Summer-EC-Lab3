#include "bsp_uart.h"

#include "usart.h"

#include <stddef.h>

typedef struct
{
    UART_HandleTypeDef *handle;
    uint8_t *rx_buffer;
    uint16_t rx_len;
    BSP_UARTRxCallback_t rx_callback;
    BSP_UARTTxCallback_t tx_callback;
    BSP_UARTErrorCallback_t error_callback;
} BSP_UART_t;

static BSP_UART_t bsp_uart[BSP_UART_NUM] = {
    [BSP_UART_USART6] = {
        &huart6,
        NULL,
        0u,
        NULL,
        NULL,
        NULL,
    },
};

static uint8_t BSP_UART_IsValid(BSP_UARTId_t uart)
{
    return (uart < BSP_UART_NUM) && (bsp_uart[uart].handle != NULL);
}

static BSP_UARTId_t BSP_UART_GetIdFromHandle(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0u; i < BSP_UART_NUM; i++)
    {
        if (bsp_uart[i].handle == huart)
        {
            return (BSP_UARTId_t)i;
        }
    }

    return BSP_UART_NUM;
}

void BSP_UART_Init(void)
{
    for (uint8_t i = 0u; i < BSP_UART_NUM; i++)
    {
        bsp_uart[i].rx_buffer = NULL;
        bsp_uart[i].rx_len = 0u;
        bsp_uart[i].rx_callback = NULL;
        bsp_uart[i].tx_callback = NULL;
        bsp_uart[i].error_callback = NULL;
    }
}

void BSP_UART_RegisterRxCallback(BSP_UARTId_t uart, BSP_UARTRxCallback_t callback)
{
    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    bsp_uart[uart].rx_callback = callback;
}

void BSP_UART_RegisterTxCallback(BSP_UARTId_t uart, BSP_UARTTxCallback_t callback)
{
    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    bsp_uart[uart].tx_callback = callback;
}

void BSP_UART_RegisterErrorCallback(BSP_UARTId_t uart, BSP_UARTErrorCallback_t callback)
{
    if (!BSP_UART_IsValid(uart))
    {
        return;
    }
 
    bsp_uart[uart].error_callback = callback;
}

HAL_StatusTypeDef BSP_UART_Send(BSP_UARTId_t uart, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(bsp_uart[uart].handle, data, len, timeout_ms);
}

HAL_StatusTypeDef BSP_UART_SendIT(BSP_UARTId_t uart, const uint8_t *data, uint16_t len)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit_IT(bsp_uart[uart].handle, data, len);
}

HAL_StatusTypeDef BSP_UART_SendDMA(BSP_UARTId_t uart, const uint8_t *data, uint16_t len)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit_DMA(bsp_uart[uart].handle, data, len);
}

HAL_StatusTypeDef BSP_UART_ReceiveIT(BSP_UARTId_t uart, uint8_t *data, uint16_t len)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    bsp_uart[uart].rx_buffer = data;
    bsp_uart[uart].rx_len = len;

    return HAL_UART_Receive_IT(bsp_uart[uart].handle, data, len);
}

HAL_StatusTypeDef BSP_UART_ReceiveDMA(BSP_UARTId_t uart, uint8_t *data, uint16_t len)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    bsp_uart[uart].rx_buffer = data;
    bsp_uart[uart].rx_len = len;

    return HAL_UART_Receive_DMA(bsp_uart[uart].handle, data, len);
}

HAL_StatusTypeDef BSP_UART_ReceiveToIdleIT(BSP_UARTId_t uart, uint8_t *data, uint16_t max_len)
{
    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (max_len == 0u))
    {
        return HAL_ERROR;
    }

    bsp_uart[uart].rx_buffer = data;
    bsp_uart[uart].rx_len = max_len;

    return HAL_UARTEx_ReceiveToIdle_IT(bsp_uart[uart].handle, data, max_len);
}

HAL_StatusTypeDef BSP_UART_ReceiveToIdleDMA(BSP_UARTId_t uart, uint8_t *data, uint16_t max_len)
{
    HAL_StatusTypeDef status;

    if ((!BSP_UART_IsValid(uart)) || (data == NULL) || (max_len == 0u))
    {
        return HAL_ERROR;
    }

    bsp_uart[uart].rx_buffer = data;
    bsp_uart[uart].rx_len = max_len;

    status = HAL_UARTEx_ReceiveToIdle_DMA(bsp_uart[uart].handle, data, max_len);

    if ((status == HAL_OK) && (bsp_uart[uart].handle->hdmarx != NULL))
    {
        __HAL_DMA_DISABLE_IT(bsp_uart[uart].handle->hdmarx, DMA_IT_HT);
    }

    return status;
}

HAL_StatusTypeDef BSP_UART_AbortReceive(BSP_UARTId_t uart)
{
    if (!BSP_UART_IsValid(uart))
    {
        return HAL_ERROR;
    }

    bsp_uart[uart].rx_buffer = NULL;
    bsp_uart[uart].rx_len = 0u;

    return HAL_UART_AbortReceive(bsp_uart[uart].handle);
}

HAL_StatusTypeDef BSP_UART_AbortTransmit(BSP_UARTId_t uart)
{
    if (!BSP_UART_IsValid(uart))
    {
        return HAL_ERROR;
    }

    return HAL_UART_AbortTransmit(bsp_uart[uart].handle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UARTId_t uart = BSP_UART_GetIdFromHandle(huart);

    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    if (bsp_uart[uart].rx_callback != NULL)
    {
        bsp_uart[uart].rx_callback(uart, bsp_uart[uart].rx_buffer, bsp_uart[uart].rx_len);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BSP_UARTId_t uart = BSP_UART_GetIdFromHandle(huart);

    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    if (bsp_uart[uart].rx_callback != NULL)
    {
        bsp_uart[uart].rx_callback(uart, bsp_uart[uart].rx_buffer, size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UARTId_t uart = BSP_UART_GetIdFromHandle(huart);

    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    if (bsp_uart[uart].tx_callback != NULL)
    {
        bsp_uart[uart].tx_callback(uart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_UARTId_t uart = BSP_UART_GetIdFromHandle(huart);

    if (!BSP_UART_IsValid(uart))
    {
        return;
    }

    if (bsp_uart[uart].error_callback != NULL)
    {
        bsp_uart[uart].error_callback(uart, huart->ErrorCode);
    }
}
