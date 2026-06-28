#include "bsp_encoder.h"

#include "tim.h"

/** @brief Maximum positive speed value returned by Encoder_CalcSpeed(). */
#define ENCODER_SPEED_MAX 2147483647LL
/** @brief Minimum negative speed value returned by Encoder_CalcSpeed(). */
#define ENCODER_SPEED_MIN (-2147483647LL - 1LL)

/**
 * @brief Internal low-level encoder runtime data.
 */
typedef struct
{
    /** HAL timer handle configured in encoder mode. */
    TIM_HandleTypeDef *htim;
    /** Timer counter maximum value, used to distinguish 16-bit and 32-bit counters. */
    uint32_t max_cnt;
    /** Previous raw timer counter value. */
    uint32_t last_cnt;
    /** Latest signed counter delta. */
    int32_t delta_cnt;
    /** Latest speed in encoder timer counts per second. */
    int32_t speed_cnt_per_s;
} Encoder_t;

/** @brief Encoder table matched to PWM_EncoderId_t. */
static Encoder_t encoder[PWM_ENCODER_NUM] = {
    [PWM_ENCODER_CH1] = {&htim1, 0x0000FFFFu, 0u, 0, 0},
    [PWM_ENCODER_CH2] = {&htim2, 0xFFFFFFFFu, 0u, 0, 0},
    [PWM_ENCODER_CH3] = {&htim3, 0x0000FFFFu, 0u, 0, 0},
    [PWM_ENCODER_CH4] = {&htim4, 0x0000FFFFu, 0u, 0, 0},
};

/**
 * @brief Calculate signed counter delta and handle one wraparound.
 *
 * @param now_cnt Current raw counter value.
 * @param last_cnt Previous raw counter value.
 * @param max_cnt Timer counter maximum value.
 * @return Signed count delta.
 *
 * @note The sampling period must be short enough that the count change is less
 *       than half of the timer range.
 */
static int32_t Encoder_GetDelta(uint32_t now_cnt, uint32_t last_cnt, uint32_t max_cnt)
{
    if (max_cnt == 0x0000FFFFu)
    {
        return (int32_t)(int16_t)((uint16_t)(now_cnt - last_cnt));
    }

    return (int32_t)(now_cnt - last_cnt);
}

/**
 * @brief Convert count delta over time to counts per second.
 *
 * @param delta_cnt Signed count delta.
 * @param dt_ms Sampling period in milliseconds.
 * @return Speed in encoder timer counts per second.
 */
static int32_t Encoder_CalcSpeed(int32_t delta_cnt, uint32_t dt_ms)
{
    int64_t speed = ((int64_t)delta_cnt * 1000) / (int64_t)dt_ms;

    if (speed > ENCODER_SPEED_MAX)
    {
        return (int32_t)ENCODER_SPEED_MAX;
    }

    if (speed < ENCODER_SPEED_MIN)
    {
        return (int32_t)ENCODER_SPEED_MIN;
    }

    return (int32_t)speed;
}

void PWM_EncoderInit(void)
{
    for (uint8_t i = 0; i < PWM_ENCODER_NUM; i++)
    {
        HAL_TIM_Encoder_Start(encoder[i].htim, TIM_CHANNEL_ALL);
        encoder[i].last_cnt = __HAL_TIM_GET_COUNTER(encoder[i].htim);
        encoder[i].delta_cnt = 0;
        encoder[i].speed_cnt_per_s = 0;
    }
}

void PWM_EncoderUpdate(uint32_t dt_ms)
{
    if (dt_ms == 0u)
    {
        return;
    }

    for (uint8_t i = 0; i < PWM_ENCODER_NUM; i++)
    {
        uint32_t now_cnt = __HAL_TIM_GET_COUNTER(encoder[i].htim);

        encoder[i].delta_cnt = Encoder_GetDelta(now_cnt, encoder[i].last_cnt, encoder[i].max_cnt);
        encoder[i].last_cnt = now_cnt;
        encoder[i].speed_cnt_per_s = Encoder_CalcSpeed(encoder[i].delta_cnt, dt_ms);
    }
}

int32_t PWM_GetEncoderSpeedCntPerS(PWM_EncoderId_t encoder_id)
{
    if (encoder_id >= PWM_ENCODER_NUM)
    {
        return 0;
    }

    return encoder[encoder_id].speed_cnt_per_s;
}

int32_t PWM_GetEncoderDelta(PWM_EncoderId_t encoder_id)
{
    if (encoder_id >= PWM_ENCODER_NUM)
    {
        return 0;
    }

    return encoder[encoder_id].delta_cnt;
}

PWM_EncoderState_t PWM_GetEncoderState(PWM_EncoderId_t encoder_id)
{
    PWM_EncoderState_t state = {0, 0, 0};

    if (encoder_id >= PWM_ENCODER_NUM)
    {
        return state;
    }

    state.speed_cnt_per_s = encoder[encoder_id].speed_cnt_per_s;
    state.delta_cnt = encoder[encoder_id].delta_cnt;
    state.counter = encoder[encoder_id].last_cnt;

    return state;
}
