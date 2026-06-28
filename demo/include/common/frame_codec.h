#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_CODEC_SOF              0xA5U
#define FRAME_CODEC_LEN_SIZE         1U
#define FRAME_CODEC_MSG_ID_SIZE      1U
#define FRAME_CODEC_CRC_SIZE         4U
#define FRAME_CODEC_MAX_DATA_LEN     255U
#define FRAME_CODEC_MAX_PAYLOAD_LEN  (FRAME_CODEC_MAX_DATA_LEN - FRAME_CODEC_MSG_ID_SIZE)
#define FRAME_CODEC_MAX_CRC_INPUT    (FRAME_CODEC_LEN_SIZE + FRAME_CODEC_MAX_DATA_LEN)
#define FRAME_CODEC_MAX_FRAME_LEN    (1U + FRAME_CODEC_LEN_SIZE + FRAME_CODEC_MAX_DATA_LEN + FRAME_CODEC_CRC_SIZE)
#define FRAME_CODEC_MIN_FRAME_LEN    (1U + FRAME_CODEC_LEN_SIZE + FRAME_CODEC_MSG_ID_SIZE + FRAME_CODEC_CRC_SIZE)

typedef void (*frame_codec_callback_t)(const uint8_t *frame, uint16_t len);

typedef struct
{
    uint8_t *buffer;
    uint16_t buffer_size;
    uint16_t index;
    uint16_t expected_len;
    uint32_t crc_error_count;
    frame_codec_callback_t on_frame;
} frame_decoder_t;

bool FrameDecoder_Init(frame_decoder_t *decoder,
                       uint8_t *buffer,
                       uint16_t buffer_size,
                       frame_codec_callback_t on_frame);

void FrameDecoder_Reset(frame_decoder_t *decoder);
void FrameDecoder_InputByte(frame_decoder_t *decoder, uint8_t byte);
void FrameDecoder_Input(frame_decoder_t *decoder, const uint8_t *data, size_t len);

bool FrameCodec_Encode(uint8_t msg_id,
                       const uint8_t *payload,
                       uint8_t payload_len,
                       uint8_t *out_frame,
                       uint16_t out_size,
                       uint16_t *out_len);

#ifdef __cplusplus
}
#endif
