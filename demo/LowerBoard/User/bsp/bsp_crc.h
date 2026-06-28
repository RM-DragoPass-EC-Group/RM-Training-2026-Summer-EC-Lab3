/**
 * @file bsp_crc.h
 * @brief BSP hardware CRC interface based on CubeMX-configured CRC.
 */

#ifndef __BSP_CRC_H__
#define __BSP_CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Initialize BSP CRC runtime state.
 *
 * Call this after MX_CRC_Init(). CubeMX owns the hardware configuration.
 */
void BSP_CRC_Init(void);

/**
 * @brief Calculate hardware CRC32 from 32-bit words.
 *
 * @param data Input data in 32-bit words.
 * @param word_len Number of 32-bit words.
 * @return CRC32 value calculated by STM32 hardware CRC.
 */
uint32_t BSP_CRC_CalculateWords(const uint32_t *data, uint32_t word_len);

/**
 * @brief Calculate hardware CRC32 from bytes.
 *
 * STM32F4 hardware CRC consumes 32-bit words through HAL_CRC_Calculate(), so
 * byte_len must be a multiple of 4.
 *
 * @param data Input bytes.
 * @param byte_len Number of bytes. Must be a multiple of 4.
 * @param crc_out Output CRC32 pointer.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef BSP_CRC_CalculateBytes(const uint8_t *data, uint16_t byte_len, uint32_t *crc_out);

/**
 * @brief Verify hardware CRC32 from bytes.
 *
 * @param data Input bytes excluding the received CRC field.
 * @param byte_len Number of bytes excluding the received CRC field.
 * @param expected_crc Received CRC32.
 * @return 1 when CRC matches, otherwise 0.
 */
uint8_t BSP_CRC_VerifyBytes(const uint8_t *data, uint16_t byte_len, uint32_t expected_crc);

#ifdef __cplusplus
}
#endif

#endif
