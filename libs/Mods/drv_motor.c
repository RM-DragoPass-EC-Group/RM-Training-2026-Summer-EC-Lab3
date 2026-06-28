#include "drv_motor.h"

#include "bsp/bsp_encoder.h"
#include "drv_tb6612.h"

/**
 * @brief Motor runtime data and BSP encoder mapping.
 */
typedef struct
{
    /** Low-level BSP encoder channel used by this motor. */
    PWM_EncoderId_t encoder_id;
    /** Encoder counts per output-shaft revolution. */
    float cnt_per_output_rev;
    /** Cached speed values. */
    MotorSpeed_t speed;
} Motor_t;

static TB6612_Channel_t Motor_GetDriveChannel(MotorId_t motor_id)
{
    static const TB6612_Channel_t channel_map[MOTOR_NUM] = {
        [MOTOR_FL] = TB6612_CH_FL,
        [MOTOR_FR] = TB6612_CH_FR,
        [MOTOR_RL] = TB6612_CH_RL,
        [MOTOR_RR] = TB6612_CH_RR,
    };

    return channel_map[motor_id];
}

static TB6612_Dir_t Motor_GetDriveDirection(MotorOutputMode_t mode)
{
    switch (mode)
    {
        case MOTOR_OUTPUT_FORWARD:
            return TB6612_DIR_FORWARD;

        case MOTOR_OUTPUT_REVERSE:
            return TB6612_DIR_REVERSE;

        case MOTOR_OUTPUT_BRAKE:
            return TB6612_DIR_BRAKE;

        case MOTOR_OUTPUT_COAST:
        default:
            return TB6612_DIR_STOP;
    }
}

static uint8_t Motor_IsOutputReversed(MotorId_t motor_id)
{
    static const uint8_t reverse_map[MOTOR_NUM] = {
        [MOTOR_FL] = MOTOR_FL_OUTPUT_REVERSE,
        [MOTOR_FR] = MOTOR_FR_OUTPUT_REVERSE,
        [MOTOR_RL] = MOTOR_RL_OUTPUT_REVERSE,
        [MOTOR_RR] = MOTOR_RR_OUTPUT_REVERSE,
    };

    return reverse_map[motor_id] != 0u;
}

static uint8_t Motor_IsEncoderReversed(MotorId_t motor_id)
{
    static const uint8_t reverse_map[MOTOR_NUM] = {
        [MOTOR_FL] = MOTOR_FL_ENCODER_REVERSE,
        [MOTOR_FR] = MOTOR_FR_ENCODER_REVERSE,
        [MOTOR_RL] = MOTOR_RL_ENCODER_REVERSE,
        [MOTOR_RR] = MOTOR_RR_ENCODER_REVERSE,
    };

    return reverse_map[motor_id] != 0u;
}

static MotorOutputMode_t Motor_ApplyOutputReverse(MotorId_t motor_id, MotorOutputMode_t mode)
{
    if (!Motor_IsOutputReversed(motor_id))
    {
        return mode;
    }

    if (mode == MOTOR_OUTPUT_FORWARD)
    {
        return MOTOR_OUTPUT_REVERSE;
    }

    if (mode == MOTOR_OUTPUT_REVERSE)
    {
        return MOTOR_OUTPUT_FORWARD;
    }

    return mode;
}

/** @brief Motor table matched to MotorId_t. */
static Motor_t motor[MOTOR_NUM] = {
    [MOTOR_FL] = {
        PWM_ENCODER_CH1,
        MOTOR_ENCODER_CNT_PER_OUTPUT_REV(MOTOR_FL_ENCODER_RESOLUTION, MOTOR_FL_REDUCTION_RATIO),
        {0, 0.0f, 0, 0},
    },
    [MOTOR_FR] = {
        PWM_ENCODER_CH2,
        MOTOR_ENCODER_CNT_PER_OUTPUT_REV(MOTOR_FR_ENCODER_RESOLUTION, MOTOR_FR_REDUCTION_RATIO),
        {0, 0.0f, 0, 0},
    },
    [MOTOR_RL] = {
        PWM_ENCODER_CH4,
        MOTOR_ENCODER_CNT_PER_OUTPUT_REV(MOTOR_RL_ENCODER_RESOLUTION, MOTOR_RL_REDUCTION_RATIO),
        {0, 0.0f, 0, 0},
    },
    [MOTOR_RR] = {
        PWM_ENCODER_CH3,
        MOTOR_ENCODER_CNT_PER_OUTPUT_REV(MOTOR_RR_ENCODER_RESOLUTION, MOTOR_RR_REDUCTION_RATIO),
        {0, 0.0f, 0, 0},
    },
};

/**
 * @brief Convert encoder counts per second to output-shaft rpm.
 *
 * @param speed_cnt_per_s Encoder speed in counts per second.
 * @param cnt_per_output_rev Encoder counts per output-shaft revolution.
 * @return Output-shaft speed in rpm.
 */
static float Motor_CalcRPM(int32_t speed_cnt_per_s, float cnt_per_output_rev)
{
    if (cnt_per_output_rev <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)speed_cnt_per_s * 60.0f) / cnt_per_output_rev;
}

void Motor_Init(void)
{
    PWM_EncoderInit();
    TB6612_Init();

    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        motor[i].speed.speed_cnt_per_s = 0;
        motor[i].speed.speed_rpm = 0.0f;
        motor[i].speed.delta_cnt = 0;
        motor[i].speed.counter = 0;
    }
}

HAL_StatusTypeDef Motor_SetOutput(MotorId_t motor_id, MotorOutputMode_t mode, uint16_t duty)
{
    TB6612_Channel_t channel;
    TB6612_Dir_t direction;

    if (motor_id >= MOTOR_NUM)
    {
        return HAL_ERROR;
    }

    channel = Motor_GetDriveChannel(motor_id);
    mode = Motor_ApplyOutputReverse(motor_id, mode);
    direction = Motor_GetDriveDirection(mode);

    if (mode == MOTOR_OUTPUT_COAST)
    {
        TB6612_SetDuty(channel, 0u);
        TB6612_SetDirection(channel, direction);
        return HAL_OK;
    }

    if (mode == MOTOR_OUTPUT_BRAKE)
    {
        TB6612_SetDirection(channel, direction);
        TB6612_SetDuty(channel, TB6612_GetMaxDuty());
        return HAL_OK;
    }

    TB6612_SetDirection(channel, direction);
    TB6612_SetDuty(channel, duty);
    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetOutputPercent(MotorId_t motor_id, float speed_percent)
{
    MotorOutputMode_t mode = MOTOR_OUTPUT_FORWARD;
    uint16_t max_duty = (uint16_t)TB6612_GetMaxDuty();
    uint16_t duty;

    if (speed_percent < 0.0f)
    {
        speed_percent = -speed_percent;
        mode = MOTOR_OUTPUT_REVERSE;
    }

    if (speed_percent > 100.0f)
    {
        speed_percent = 100.0f;
    }

    if (speed_percent == 0.0f)
    {
        return Motor_Stop(motor_id);
    }

    duty = (uint16_t)(((float)max_duty * speed_percent) / 100.0f);

    return Motor_SetOutput(motor_id, mode, duty);
}

HAL_StatusTypeDef Motor_Stop(MotorId_t motor_id)
{
    return Motor_SetOutput(motor_id, MOTOR_OUTPUT_COAST, 0u);
}

HAL_StatusTypeDef Motor_Brake(MotorId_t motor_id)
{
    return Motor_SetOutput(motor_id, MOTOR_OUTPUT_BRAKE, (uint16_t)TB6612_GetMaxDuty());
}

HAL_StatusTypeDef Motor_StopAll(void)
{
    TB6612_StopAll();
    return HAL_OK;
}

void Motor_UpdateSpeed(uint32_t dt_ms)
{
    if (dt_ms == 0u)
    {
        return;
    }

    PWM_EncoderUpdate(dt_ms);

    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        PWM_EncoderState_t encoder_state = PWM_GetEncoderState(motor[i].encoder_id);
        int32_t speed_cnt_per_s = encoder_state.speed_cnt_per_s;
        int32_t delta_cnt = encoder_state.delta_cnt;

        if (Motor_IsEncoderReversed((MotorId_t)i))
        {
            speed_cnt_per_s = -speed_cnt_per_s;
            delta_cnt = -delta_cnt;
        }

        motor[i].speed.speed_cnt_per_s = speed_cnt_per_s;
        motor[i].speed.speed_rpm = Motor_CalcRPM(speed_cnt_per_s, motor[i].cnt_per_output_rev);
        motor[i].speed.delta_cnt = delta_cnt;
        motor[i].speed.counter = encoder_state.counter;
    }
}

float Motor_GetSpeedRPM(MotorId_t motor_id)
{
    if (motor_id >= MOTOR_NUM)
    {
        return 0.0f;
    }

    return motor[motor_id].speed.speed_rpm;
}

int32_t Motor_GetSpeedCntPerS(MotorId_t motor_id)
{
    if (motor_id >= MOTOR_NUM)
    {
        return 0;
    }

    return motor[motor_id].speed.speed_cnt_per_s;
}

MotorSpeed_t Motor_GetSpeedState(MotorId_t motor_id)
{
    MotorSpeed_t state = {0, 0.0f, 0, 0};

    if (motor_id >= MOTOR_NUM)
    {
        return state;
    }

    return motor[motor_id].speed;
}
