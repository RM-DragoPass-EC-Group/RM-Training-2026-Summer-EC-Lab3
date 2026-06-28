#include "frame_codec.h"

#include <string.h>

#include "crc.h"

static uint32_t s_crc_words[(FRAME_CODEC_MAX_CRC_INPUT + 3U) / 4U];

static uint32_t read_le32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t frame_total_len(uint8_t data_len)
{
    return (uint16_t)(1U + FRAME_CODEC_LEN_SIZE + data_len + FRAME_CODEC_CRC_SIZE);
}

static uint32_t calculate_crc32(const uint8_t *data, uint16_t len)
{
    const uint16_t word_count = (uint16_t)((len + 3U) / 4U);

    if ((data == NULL) || (len == 0U) || (len > FRAME_CODEC_MAX_CRC_INPUT))
    {
        return 0U;
    }

    (void)memset(s_crc_words, 0, sizeof(s_crc_words));

    for (uint16_t i = 0U; i < len; ++i)
    {
        s_crc_words[i / 4U] |= ((uint32_t)data[i]) << ((i % 4U) * 8U);
    }

    return HAL_CRC_Calculate(&hcrc, s_crc_words, word_count);
}

static bool check_frame_crc(const frame_decoder_t *decoder)
{
    const uint8_t data_len = decoder->buffer[1];
    const uint16_t crc_offset = (uint16_t)(1U + FRAME_CODEC_LEN_SIZE + data_len);
    const uint32_t expected_crc = read_le32(&decoder->buffer[crc_offset]);
    const uint32_t actual_crc = calculate_crc32(&decoder->buffer[1],
                                                (uint16_t)(FRAME_CODEC_LEN_SIZE + data_len));

    return actual_crc == expected_crc;
}

bool FrameDecoder_Init(frame_decoder_t *decoder,
                       uint8_t *buffer,
                       uint16_t buffer_size,
                       frame_codec_callback_t on_frame)
{
    if ((decoder == NULL) ||
        (buffer == NULL) ||
        (buffer_size < FRAME_CODEC_MIN_FRAME_LEN) ||
        (on_frame == NULL))
    {
        return false;
    }

    decoder->buffer = buffer;
    decoder->buffer_size = buffer_size;
    decoder->on_frame = on_frame;
    decoder->crc_error_count = 0U;
    FrameDecoder_Reset(decoder);

    return true;
}

void FrameDecoder_Reset(frame_decoder_t *decoder)
{
    if (decoder == NULL)
    {
        return;
    }

    decoder->index = 0U;
    decoder->expected_len = 0U;
}

void FrameDecoder_InputByte(frame_decoder_t *decoder, uint8_t byte)
{
    if ((decoder == NULL) || (decoder->buffer == NULL) || (decoder->on_frame == NULL))
    {
        return;
    }

    if (decoder->index == 0U)
    {
        if (byte == FRAME_CODEC_SOF)
        {
            decoder->buffer[0] = byte;
            decoder->index = 1U;
        }
        return;
    }

    if (decoder->index == 1U)
    {
        const uint8_t data_len = byte;
        const uint16_t total_len = frame_total_len(data_len);

        if ((data_len < FRAME_CODEC_MSG_ID_SIZE) || (total_len > decoder->buffer_size))
        {
            FrameDecoder_Reset(decoder);
            return;
        }

        decoder->buffer[1] = byte;
        decoder->expected_len = total_len;
        decoder->index = 2U;
        return;
    }

    decoder->buffer[decoder->index] = byte;
    decoder->index++;

    if (decoder->index < decoder->expected_len)
    {
        return;
    }

    if (check_frame_crc(decoder))
    {
        decoder->on_frame(decoder->buffer, decoder->expected_len);
    }
    else
    {
        decoder->crc_error_count++;
    }

    FrameDecoder_Reset(decoder);
}

void FrameDecoder_Input(frame_decoder_t *decoder, const uint8_t *data, size_t len)
{
    if ((decoder == NULL) || (data == NULL))
    {
        return;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        FrameDecoder_InputByte(decoder, data[i]);
    }
}

bool FrameCodec_Encode(uint8_t msg_id,
                       const uint8_t *payload,
                       uint8_t payload_len,
                       uint8_t *out_frame,
                       uint16_t out_size,
                       uint16_t *out_len)
{
    const uint8_t data_len = (uint8_t)(FRAME_CODEC_MSG_ID_SIZE + payload_len);
    const uint16_t total_len = frame_total_len(data_len);
    uint32_t crc32;

    if ((out_frame == NULL) ||
        (out_len == NULL) ||
        (payload_len > FRAME_CODEC_MAX_PAYLOAD_LEN) ||
        ((payload == NULL) && (payload_len != 0U)) ||
        (out_size < total_len))
    {
        return false;
    }

    out_frame[0] = FRAME_CODEC_SOF;
    out_frame[1] = data_len;
    out_frame[2] = msg_id;

    if (payload_len != 0U)
    {
        (void)memcpy(&out_frame[3], payload, payload_len);
    }

    crc32 = calculate_crc32(&out_frame[1], (uint16_t)(FRAME_CODEC_LEN_SIZE + data_len));
    write_le32(&out_frame[1U + FRAME_CODEC_LEN_SIZE + data_len], crc32);
    *out_len = total_len;

    return true;
}
