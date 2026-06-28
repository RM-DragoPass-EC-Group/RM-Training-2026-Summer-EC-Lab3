#include "bsp_crc.h"

#include "crc.h"

#include <stddef.h>

void BSP_CRC_Init(void)
{
    if (hcrc.Instance != CRC)
    {
        MX_CRC_Init();
    }

    __HAL_CRC_DR_RESET(&hcrc);
}

uint32_t BSP_CRC_CalculateWords(const uint32_t *data, uint32_t word_len)
{
    if ((data == NULL) || (word_len == 0u))
    {
        return 0u;
    }

    return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, word_len);
}

HAL_StatusTypeDef BSP_CRC_CalculateBytes(const uint8_t *data, uint16_t byte_len, uint32_t *crc_out)
{
    if ((data == NULL) || (crc_out == NULL) || (byte_len == 0u) || ((byte_len % 4u) != 0u))
    {
        return HAL_ERROR;
    }

    *crc_out = BSP_CRC_CalculateWords((const uint32_t *)data, (uint32_t)byte_len / 4u);

    return HAL_OK;
}

uint8_t BSP_CRC_VerifyBytes(const uint8_t *data, uint16_t byte_len, uint32_t expected_crc)
{
    uint32_t crc;

    if (BSP_CRC_CalculateBytes(data, byte_len, &crc) != HAL_OK)
    {
        return 0u;
    }

    return (crc == expected_crc) ? 1u : 0u;
}
